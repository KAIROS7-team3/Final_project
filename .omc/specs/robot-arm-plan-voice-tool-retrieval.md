# Robot Arm Project Plan: Voice-Commanded Tool Retrieval System

> Status: pending approval
> Updated: 2026-05-21 (added PLC, DB/FOD, Gemma 4, ROS2 pkg structure, Track C architecture)

---

## 1. Project Summary

| Item | Detail |
|------|--------|
| Robot Arm | Doosan Robotics e0509 (collaborative robot) |
| Gripper | ROBOTIS RH-P12-RN |
| Camera | Intel RealSense D455f |
| Wrist F/T Sensor | Required — model TBD |
| PLC | LED status display — protocol TBD |
| Database | FOD management + tool inventory (engine TBD) |
| OS | Ubuntu 22.04 |
| Middleware | ROS2 Humble (Track A/B only) |
| ROS2 Drivers | doosan-robot2, realsense-ros |
| STT Engine | Whisper (local, GPU) — shared all tracks |
| Context LLM | Gemma 4 (local) — Track A/B intent + DB check |
| Planning LLM | Larger model TBD — Track C full planning |
| Primary Machine | Vector 16 HX AI A2XWIG |
| Secondary Machine | HP ProBook 450 G10 (dev / monitoring) |
| Development Mode | Simulation + real hardware parallel |

**Core Task:** Operator gives a voice command naming a tool. The robot fetches it from a
semi-fixed toolbox and hands it over. On return command, the robot receives the tool and
replaces it. All tool events are recorded in DB for FOD management. PLC drives LED status.

**Tool Inventory:** 9 tools (3 screwdrivers, 3 wrenches, 3 pliers) — extensible via YAML.

**Track Strategy:** A/B/C share all hardware inputs and drivers. Differences are isolated to
the decision and motion layers only, enabling direct comparison of approaches.

| Track | Context Understanding | Decision | Motion |
|-------|-----------------------|----------|--------|
| **A — DSR** | Gemma 4 + DB check | Behavior Tree | DSR coordinate control |
| **B — RL** | Gemma 4 + DB check | Behavior Tree | RL policy |
| **C — VLA** | LLM planner (full) | Task Executor | DSR or RL |

---

## 2. Architecture Design

### Shared vs Track-Specific

```
╔══════════════════════════════════════════════════════════════╗
║                    SHARED (all tracks)                       ║
║  Microphone  ·  RealSense D455f  ·  F/T Sensor              ║
║  Whisper STT  ·  doosan-robot2  ·  realsense-ros            ║
║  Gripper Driver  ·  Unit Action Library (pure Python)        ║
║  HAL (ArmInterface · GripperInterface · FTSensorInterface)   ║
║  DB (FOD + tool inventory)  ·  PLC (LED status)             ║
╠══════════════════════════╦═══════════════════════════════════╣
║  TRACK A / B             ║  TRACK C                          ║
║  YOLOv8                  ║  Raw RGB-D → LLM direct input     ║
║  6D Pose Estimation      ║  (YOLOv8 not used)               ║
║  Object Tracker          ║                                   ║
║  Gemma 4 + DB check      ║  LLM Planner (full)              ║
║  BT                      ║  Plan Generator                  ║
║  DSR / RL                ║  Safety Validator                ║
║                          ║  Task Executor                   ║
╚══════════════════════════╩═══════════════════════════════════╝
```

### Full Software Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                        INPUT LAYER (shared)                     │
│      Microphone → Whisper STT   │   RealSense D455f → RGB-D    │
├──────────────────────────────────┬──────────────────────────────┤
│  PERCEPTION — Track A/B          │  PERCEPTION — Track C        │
│  YOLOv8 · 6D Pose · Tracker      │  Raw RGB-D → LLM direct      │
│                                  │  (YOLOv8 not used)           │
├──────────────────────────────────┴──────────────────────────────┤
│                      [split below]                              │
├──────────────────────┬──────────────────────────────────────────┤
│  TRACK A / B         │  TRACK C                                 │
│  ┌────────────────┐  │  ┌─ System 2 (~1Hz) ──────────────────┐ │
│  │ Gemma 4 LLM    │  │  │ LLM Planner (full planning)        │ │
│  │ Intent + DB ck │  │  │ Plan Generator → JSON Plan         │ │
│  └───────┬────────┘  │  └──────────────────┬─────────────────┘ │
│  ┌───────▼────────┐  │  ┌─ System 1 (~10Hz)─▼─────────────────┐│
│  │ Behavior Tree  │  │  │ Safety Validator                     ││
│  │ py_trees       │  │  │ Task Executor                        ││
│  └───────┬────────┘  │  └──────────────────┬────────────────── ┘│
├──────────▼───────────┴──────────────────────▼───────────────────┤
│              UNIT ACTION LIBRARY — pure Python module (shared)  │
│   move_to · scan · grasp · release · handover_wait              │
│   return_to_slot · check_tool_presence · emergency_stop         │
├──────────────────────┬──────────────────────────────────────────┤
│  Track A             │  Track B + C                             │
│  DSR Controller      │  RL Policy Network                       │
├──────────────────────┴──────────────────────────────────────────┤
│                         HAL (shared)                            │
│       ArmInterface · GripperInterface · FTSensorInterface       │
├─────────────────────────────────────────────────────────────────┤
│                   CROSS-CUTTING (shared)                        │
│   DB Node (FOD / tool inventory)  ·  PLC Node (LED status)     │
├─────────────────────────────────────────────────────────────────┤
│                      HARDWARE LAYER                             │
│   Doosan e0509 · RH-P12-RN · D455f · F/T · PLC · DB           │
└─────────────────────────────────────────────────────────────────┘
```

### Track A/B Decision Flow (Gemma 4 + DB)

```
Whisper STT text
      │
      ▼
┌─────────────────────────────────┐
│         Gemma 4 (local LLM)     │
│  Input: STT text + DB snapshot  │
│  • Intent classification        │
│  • Tool ID resolution           │
│  • DB feasibility check:        │
│    - tool present in slot?      │
│    - tool not currently out?    │
│    - tool not flagged FOD?      │
│  Output: {action, tool_id,      │
│           feasible, reason}     │
└──────────────┬──────────────────┘
               │
       ┌───────▼───────┐
       │  feasible?    │
       │  YES  │  NO   │
       ▼       ▼       │
     BT     Inform    │
   execute  operator  │
            + log DB  │
```

### Track C Architecture (Single Python File)

Track C intentionally bypasses ROS2 middleware. All logic runs in one process:

```python
# track_c_vla.py — self-contained entry point
import asyncio
from unit_actions import UnitActions          # shared pure-Python module
from hal import ArmDriver, GripperDriver      # shared HAL
from perception import PerceptionPipeline     # shared perception
from db import DBClient                       # shared DB client
from plc import PLCClient                     # shared PLC client

async def main():
    # Shared components initialized directly (no ROS2)
    hal = ArmDriver(backend="dsr")            # or "rl"
    actions = UnitActions(hal)
    perception = PerceptionPipeline()
    db = DBClient()
    plc = PLCClient()

    # VLA loop
    while True:
        text = await stt.listen()
        context = build_context(text, perception, hal, db)
        plan = await llm_planner.plan(context)      # System 2
        if safety_validator.check(plan):
            await task_executor.run(plan, actions)  # System 1
```

**Unit Action Library for Track C — Decision: YES, use it**
- Extract unit actions as a pure Python module (`unit_actions/`) independent of ROS2
- Track A/B: ROS2 `unit_action_server` wraps this module (thin ROS2 adapter)
- Track C: imports and calls the module directly
- Result: identical business logic, different transport layer

```
unit_actions/               ← pure Python, no ROS2
├── __init__.py
├── arm_actions.py          move_to, scan
├── gripper_actions.py      grasp, release
├── handover_actions.py     handover_wait
└── slot_actions.py         return_to_slot, check_tool_presence

Track A/B ROS2:  unit_action_server.py  (rclpy wrapper around unit_actions/)
Track C:         from unit_actions import UnitActions  (direct import)
```

### ROS2 Package Structure (Track A/B)

```
ros2_ws/src/
├── interfaces/          Custom msg / srv / action definitions
│   ├── msg/
│   │   ├── ToolStatus.msg       # tool_id, slot, status, timestamp
│   │   ├── PLCStatus.msg        # led_color, led_mode, system_state
│   │   ├── RobotStatus.msg      # is_moving (audio gating)
│   │   ├── Intent.msg           # intent_type, tool_id, confidence, raw_utterance
│   │   └── HandoverEvent.msg    # v2.0+ only (S-6 — not in v1.0)
│   ├── srv/
│   │   ├── CheckToolFeasibility.srv  # query: intent+tool_id → feasible+reason
│   │   └── UpdateToolStatus.srv      # write tool status to DB
│   └── action/                  # one per behavior — typed parameters per action
│       ├── MoveToPose.action         # target_pose
│       ├── Grasp.action              # tool_id + approach_direction + force
│       ├── Release.action            # (no params)
│       ├── PlaceAtStaging.action     # tool_id
│       ├── PickFromStaging.action    # tool_id
│       └── ReturnToSlot.action       # tool_id + slot_row/col
│   # Full schema: docs/interfaces.md
│
├── voice/               STT + Gemma 4 context LLM
│   ├── whisper_node.py       audio → raw text (/voice/raw_text)
│   └── gemma_intent_node.py  text + DB → {action, tool_id, feasible}
│
├── vision/              Perception pipeline
│   ├── yolo_node.py          RGB → detections (/vision/detections)
│   ├── pose_node.py          detections + depth → 6D poses
│   ├── tracker_node.py       multi-object tracking
│   └── context_builder.py    [Track C feed] scene JSON publisher
│
├── orchestrator/        Decision + task management
│   ├── behavior_manager.py   BT tick loop (Track A/B)
│   ├── bt_nodes/             FetchTool, ReturnTool, Handover subtrees
│   └── unit_action_server.py ROS2 wrapper around unit_actions/
│
├── db/                  Database interface + FOD management
│   ├── db_node.py            ROS2 node exposing DB services/topics
│   ├── fod_monitor.py        monitors unaccounted tools, raises alerts
│   └── tool_inventory.py     CRUD for tool records
│
├── motion/              Motion control + handover detection
│   ├── dsr_controller.py     Track A: DSR coordinate control
│   ├── rl_policy_node.py     Track B: RL policy inference
│   ├── grasp_planner.py      tool pose → grasp candidate
│   └── handover_detector.py  F/T + vision → HandoverEvent
│
└── plc/                 PLC communication + LED control
    ├── plc_node.py           PLC protocol driver (Modbus/EtherNet IP TBD)
    └── led_state_mapper.py   system state → LED command
```

### DB Schema (FOD Management)

```sql
-- Tool inventory
CREATE TABLE tools (
    tool_id     TEXT PRIMARY KEY,   -- e.g. "screwdriver_phillips_small"
    label       TEXT,               -- "Phillips #1"
    slot_row    INT,
    slot_col    INT,
    grasp_axis  TEXT                -- "shaft" | "handle"
);

-- Tool status (current state)
CREATE TABLE tool_status (
    tool_id     TEXT PRIMARY KEY REFERENCES tools(tool_id),
    status      TEXT,               -- 'in_slot' | 'out' | 'missing' | 'fod_alert'
    updated_at  REAL                -- Unix timestamp
);

-- Event log (audit trail for FOD)
CREATE TABLE tool_events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    tool_id     TEXT,
    event_type  TEXT,               -- 'fetch' | 'return' | 'missing' | 'fod_alert'
    operator_id TEXT,
    timestamp   REAL,
    track       TEXT,               -- 'A' | 'B' | 'C'
    notes       TEXT
);
```

### PLC LED State Mapping

| System State | LED Color | Mode |
|-------------|-----------|------|
| Idle / Ready | Green | Solid |
| STT listening | Blue | Pulse |
| LLM processing | Yellow | Pulse |
| Robot moving | Blue | Fast pulse |
| Handover waiting | Cyan | Solid |
| Return in progress | Purple | Pulse |
| Tool missing / FOD alert | Orange | Flash |
| Error | Red | Flash |
| E-Stop active | Red | Solid |

### HAL Design

```python
class ArmInterface(ABC):
    @abstractmethod
    def move_to_pose(self, pose: Pose, speed: float) -> bool: ...
    @abstractmethod
    def get_joint_states(self) -> JointState: ...
    @abstractmethod
    def stop(self) -> None: ...

class GripperInterface(ABC):
    @abstractmethod
    def open(self, width: float) -> bool: ...
    @abstractmethod
    def close(self, force: float) -> bool: ...
    @abstractmethod
    def get_state(self) -> GripperState: ...

class FTSensorInterface(ABC):
    @abstractmethod
    def get_wrench(self) -> Wrench: ...
    @abstractmethod
    def zero(self) -> None: ...

class DBInterface(ABC):
    @abstractmethod
    def get_tool_status(self, tool_id: str) -> ToolStatus: ...
    @abstractmethod
    def log_event(self, event: ToolEvent) -> None: ...

class PLCInterface(ABC):
    @abstractmethod
    def set_led_state(self, state: LEDState) -> None: ...
```

---

## 3. Safety Architecture

**Current scope (v1.0):** Doosan e0509 built-in + software watchdog + PLC visual feedback

| Level | Mechanism | Implementation |
|-------|-----------|----------------|
| Level 0 | Hardware E-Stop | Doosan teach pendant + PLC E-stop relay |
| Level 1 | Built-in collision detection | Doosan e0509 cobot safety controller |
| Level 2 | Software watchdog | `SafetyWatchdog` node — heartbeat 500 ms |
| Level 3 | Workspace limits | DSR joint/Cartesian soft limits |
| Level 4 | PLC LED feedback | Operator sees system state at all times |

**Track C additional gate:**
```
LLM plan JSON → SafetyValidator.check(plan) → [PASS] → UnitActions
                                             → [FAIL] → reject + log DB + PLC red flash
```

**Handover Safety Protocol:**
```
Approach → speed 30% → force-compliant mode
→ F/T Δforce > 3N AND hand visible → HandoverEvent
→ open gripper → retreat → log DB → PLC green
```

**Planned extensions (v2.0+):**
- Voice E-stop ("멈춰" → /arm/stop)
- Vision safety zone monitoring

---

## 4. Perception Pipeline

### Sensor Stack

| Sensor | Model | Rate | Role |
|--------|-------|------|------|
| RGB-D | RealSense D455f | 30 Hz | Detection + pose |
| Wrist F/T | TBD | ≥ 100 Hz | Handover detection |
| Gripper current | RH-P12-RN built-in | — | Secondary grasp confirm |

### Tool Recognition (Track A/B only)

```
D455f RGB → YOLOv8 → tool ID + bbox
D455f Depth → point cloud → ICP / FoundationPose → 6D pose
```

Tool classes registered in `tools.yaml`. Adding tool = YAML entry + YOLOv8 retrain.

### Track C Vision Input

Track C does **not** use YOLOv8. Raw RGB-D frames are fed directly into the LLM planner as
vision-language input. The LLM handles tool localization and planning in a single pass.

```
D455f RGB-D → LLM Planner (vision-language input)
              → tool localization + intent + plan JSON (unified)
```

### Handover Detection

```
F/T: Δforce > 3N  AND  Camera: hand keypoints near gripper
Both true for 200ms → HandoverEvent → DB log → PLC state update
```

### Timing Budget

| Stage | Track A/B | Track C |
|-------|-----------|---------|
| Whisper STT | < 500 ms | < 500 ms |
| Gemma 4 intent + DB check | < 800 ms | — |
| YOLOv8 + pose | < 150 ms | N/A (not used) |
| Context build (raw image) | — | < 100 ms |
| LLM inference (vision + plan) | — | 500 ms – 2 s |
| **Voice → motion start** | **~1.5 s** | **~2.5–3.5 s** |

---

## 5. Development Phases

### Phase 0: Environment + Shared Foundation (Week 1–2)
- [ ] ROS2 Humble workspace: doosan-robot2 + realsense-ros
- [ ] `interfaces` package: all custom msg/srv/action definitions
- [ ] Docker dev container: GPU passthrough (Whisper + YOLOv8 + Gemma 4 + LLM)
- [ ] Doosan e0509 URDF/XACRO validation + Gazebo scene
- [ ] HAL stubs: `SimulatedArm`, `SimulatedGripper`, `SimulatedFTSensor`
- [ ] `unit_actions/` pure Python module skeleton (mock implementations)
- [ ] DB schema creation + SQLite/PostgreSQL setup
- [ ] CycloneDDS config for single-machine setup

### Phase 1: Hardware Drivers (Week 2–3)
- [ ] doosan-robot2 driver bring-up + joint state verification
- [ ] realsense-ros D455f node — RGB + depth stream validation
- [ ] RH-P12-RN gripper driver node
- [ ] F/T sensor driver node (model selection)
- [ ] PLC driver node (`plc` pkg): LED write via Modbus/EtherNet IP
- [ ] udev rules: `/dev/doosan`, `/dev/gripper`, `/dev/ft_sensor`, `/dev/plc`
- [ ] Hand-eye calibration (camera extrinsic vs end-effector)

### Phase 2: Shared Perception + Voice (Week 3–6)
- [ ] Whisper STT node (`voice` pkg)
- [ ] YOLOv8 fine-tuning on 9-tool dataset (`vision` pkg)
- [ ] 6D pose + object tracker nodes
- [ ] Semi-fixed slot correction (misalignment detection)
- [ ] Tool presence check (empty slot detection)
- [ ] `vision/context_builder.py` — scene JSON for Track C

### Phase 3: DB + PLC Integration (Week 4–5, parallel)
- [ ] `db` pkg: DB node exposing CheckToolFeasibility + UpdateToolStatus services
- [ ] FOD monitor: alert when tool status = 'out' > timeout threshold
- [ ] Tool events log (fetch, return, missing, fod_alert)
- [ ] `plc` pkg: LED state mapper wired to system FSM
- [ ] PLC integration test: all LED states verified on hardware

### Phase 4: Handover Detection (Week 5–7)
- [ ] `ft_sensor_node` + threshold event publisher (`motion` pkg)
- [ ] `handover_detector_node`: F/T + vision fusion
- [ ] Handover event → DB log + PLC LED update
- [ ] Edge case tuning: tool drop, operator hesitation, unexpected contact

### Phase 5: Track A/B — Gemma 4 + Behavior Tree (Week 6–10)

**Gemma 4 intent node (`voice` pkg):**
- [ ] Local Gemma 4 inference setup (GPU, Vector 16 HX)
- [ ] System prompt: intent classification + tool ID resolution
- [ ] DB feasibility check integration (CheckToolFeasibility service call)
- [ ] Output: `{action, tool_id, feasible, reason}` → `/voice/intent`
- [ ] Fallback: infeasible → inform operator + log DB + PLC orange flash

**Behavior Tree (`orchestrator` pkg):**
- [ ] `FetchTool` BT subtree
- [ ] `ReturnTool` BT subtree
- [ ] Error recovery subtrees (tool not found, grasp fail, handover timeout)
- [ ] Blackboard: `{active_tool_id, tool_pose, handover_state, intent}`
- [ ] DB write on each BT terminal node (fetch complete, return complete, fail)
- [ ] PLC state updates at each BT phase transition

**Motion tracks:**
- [ ] Track A: `DSRArmDriver` → `unit_action_server` (`motion` pkg)
- [ ] Track B: RL training (Isaac Sim / MuJoCo) + policy deployment node

### Phase 6: Track C — VLA Single Python File (Week 8–12)
- [ ] `track_c_vla.py` entry point: direct HAL + unit_actions imports (no ROS2)
- [ ] LLM planner model selection + local inference (larger than Gemma 4)
- [ ] Context builder (shared pure Python, no ROS2)
- [ ] Safety validator (schema + behavior rules check)
- [ ] Task executor: sequential unit_actions calls with F/T + vision monitoring
- [ ] DB client (direct, no ROS2 service)
- [ ] PLC client (direct, no ROS2 node)
- [ ] Fallback: LLM timeout / invalid plan → halt + inform + log

### Phase 7: Comparative Evaluation (Week 12–14)

Run identical test scenarios across all three tracks:

| Scenario | Track A | Track B | Track C |
|----------|---------|---------|---------|
| Fetch — known tool, clear slot | baseline | vs A | vs A |
| Fetch — ambiguous name ("드라이버") | Gemma clarify | Gemma clarify | LLM clarify |
| Fetch — tool absent (DB: out) | Gemma blocks | Gemma blocks | LLM blocks |
| Return — correct slot | baseline | vs A | vs A |
| FOD alert (tool missing > timeout) | DB/PLC alert | DB/PLC alert | DB/PLC alert |

### Phase 8: Testing (Week 2–14, parallel)
- [ ] Unit: kinematics, YAML loader, Gemma 4 intent accuracy, DB CRUD, PLC state mapping
- [ ] Unit (Track C): context builder, LLM intent, plan schema, SafetyValidator
- [ ] Integration: `launch_testing` — full voice → perception → motion pipeline (Track A/B)
- [ ] Integration (Track C): subprocess-based end-to-end test
- [ ] Simulation: BT golden-file regression + Track C plan replay
- [ ] HIL: all 9 tools × 3 fetch+return cycles per track
- [ ] Comparative: same 20-scenario suite run on all 3 tracks, metrics recorded

### Phase 9: Deployment (Week 14–15)
- [ ] systemd service: auto-start ROS2 stack + Track C script on boot
- [ ] Docker Compose: STT, perception, Gemma 4, VLA LLM, control, DB, PLC containers
- [ ] ROS2 bag recording for all sessions + DB audit trail
- [ ] HP ProBook: rqt monitoring dashboard + DB viewer + PLC LED status panel
- [ ] Log rotation + DB backup schedule

---

## 6. Test Strategy

### Testing Pyramid
Unit → Integration → Simulation → HIL → Comparative E2E

### Key Test Cases

| Test | Criteria |
|------|----------|
| Gemma 4 intent accuracy | ≥ 97% (fetch/return/ambiguous on 9 tools) |
| DB feasibility check correctness | 100% block when tool status = 'out' or 'fod_alert' |
| FOD alert latency | ≤ 30 s from tool going missing to PLC orange flash |
| PLC LED state coverage | All 9 states verified on hardware |
| Tool classification (YOLOv8) | ≥ 95% per class |
| Pose estimation error | ≤ 5 mm, ≤ 3° |
| Fetch cycle time (Track A/B) | ≤ 12 s (Gemma 4 adds ~800 ms vs original) |
| Fetch cycle time (Track C) | ≤ 15 s |
| Handover detection FP rate | ≤ 2% |
| E-stop response | ≤ 500 ms |
| Track C SafetyValidator reject rate | 100% of schema-invalid plans |

---

## 7. Security Considerations

- DDS domain isolation: dedicated `ROS_DOMAIN_ID`
- USB/PLC access: udev rules with `GROUP=robotics` (no root)
- Whisper + Gemma 4 + VLA LLM: local inference only (no data leaves machine)
- DB: file permissions restricted; audit log append-only
- PLC: isolated network segment (control plane); no public interface
- SROS2: not required for v1.0 (single-machine, lab)

---

## 8. Risks and Mitigations

| Risk | Severity | Mitigation |
|------|----------|------------|
| Gemma 4 misclassification (wrong tool_id) | High | Confidence threshold + "did you mean?" fallback |
| Gemma 4 DB check latency > 800 ms | Medium | Cache last DB snapshot; async DB query |
| LLM hallucination (Track C invalid plan) | High | SafetyValidator + action vocabulary restriction |
| PLC communication failure | Medium | Watchdog: if PLC silent > 2s, log warning; robot continues |
| DB corruption / race condition | Medium | SQLite WAL mode; single writer per table |
| FOD false positive (tool in slot but detected missing) | Medium | Vision double-check before alert; debounce 10 s |
| Sim-to-real gap (Track B RL) | High | Domain randomization; DSR fallback |
| Hand-eye calibration drift | High | Periodic recalibration; AprilTag reference board |
| F/T sensor model unavailable | Medium | Evaluate ATI Mini45, Robotiq FT 300-S, OnRobot HEX-E |
| Whisper latency spike | Low | Pin GPU clocks; profile on Vector 16 HX |

---

## 9. ADR (Architecture Decision Record)

### ADR-001: Motion Control Backend
- **Decision**: Three-track evaluation — DSR (A), RL (B), VLA (C)
- **Drivers**: Reliability, adaptiveness, natural language flexibility
- **Common HAL + unit_actions module**: enables fair comparison + code sharing
- **Follow-up**: Select primary after Phase 7 comparative evaluation

### ADR-002: Tool Recognition
- **Decision**: YOLOv8 + depth-based pose refinement
- **Why**: GPU on Vector 16 HX; YAML-driven extensibility
- **Consequences**: Dataset collection required; retrain when tools added

### ADR-003: Behavioral Framework (Track A/B)
- **Decision**: Behavior Tree (py_trees)
- **Why**: Modular subtrees, reactive fallback, each independently testable

### ADR-004: LLM Model Selection (Track C)
| Model | VRAM | Latency | Quality |
|-------|------|---------|---------|
| Qwen2.5-7B Q4 | ~6 GB | ~1 s | Good |
| Qwen2.5-14B Q4 | ~10 GB | ~1.5 s | Better |
| GPT-4o API | 0 GB | ~1–2 s | Best (internet) |
- **Follow-up**: Benchmark on Vector 16 HX before Phase 6

### ADR-005: VLA Safety Boundary
- **Decision**: LLM output always passes SafetyValidator before UnitActions
- **Why**: LLM hallucination risk in HRC environment

### ADR-006: Unit Action Library Architecture
- **Decision**: Pure Python module (`unit_actions/`), ROS2 adapter for Track A/B
- **Drivers**: Code sharing between tracks; Track C's no-middleware constraint
- **Why**: Avoids duplicating business logic; Track C stays ROS2-free
- **Consequences**: unit_actions/ must not import rclpy; tested independently of ROS2

### ADR-007: Context LLM for Track A/B
- **Decision**: Gemma 4 (local) for intent classification + DB feasibility check
- **Drivers**: Replaces rigid rule-based parser; handles ambiguous commands; DB-aware
- **Alternatives**: Rule-based parser (rejected — no ambiguity handling), GPT-4o (rejected — internet dependency)
- **Why**: Local inference preserves privacy; Gemma 4 small enough for fast inference
- **Consequences**: +~800 ms cycle time vs rule-based; requires Gemma 4 fine-tuning on tool vocabulary

### ADR-008: Database Engine
- **Decision**: TBD — SQLite (simple) vs PostgreSQL (scalable)
- **Drivers**: Single-machine deployment; FOD audit trail; concurrent read from Track A/B/C
- **Candidates**:
  - SQLite + WAL mode: no server, sufficient for lab scale
  - PostgreSQL: better concurrent writes, overkill for single machine
- **Follow-up**: Decide before Phase 3

### ADR-009: PLC Protocol
- **Decision**: TBD — Modbus RTU vs EtherNet/IP vs OPC-UA
- **Drivers**: PLC model selection, latency, ease of Python/ROS2 integration
- **Follow-up**: Decide with PLC hardware selection

---

## 10. Acceptance Criteria

**Shared (all tracks):**
- [ ] Tool classification ≥ 95% under lab lighting
- [ ] Slot return error ≤ 5 mm
- [ ] Handover detection ≥ 98% correct
- [ ] DB logs every fetch/return/FOD event with correct timestamps
- [ ] PLC LED reflects correct system state within 500 ms of state change
- [ ] FOD alert triggered within 30 s of tool going missing
- [ ] System starts on Vector 16 HX boot via systemd

**Track A/B:**
- [ ] Gemma 4 intent accuracy ≥ 97%
- [ ] Gemma 4 correctly blocks infeasible commands (tool out / FOD) 100% of time
- [ ] Voice → handover ≤ 12 s for all 9 tools
- [ ] Full BT integration test passes (all 9 tools × 3 cycles)

**Track C:**
- [ ] Voice → handover ≤ 15 s
- [ ] LLM hallucination rate ≤ 3%
- [ ] SafetyValidator rejects 100% of invalid plans
- [ ] Track C correctly queries DB and blocks infeasible commands

---

## 11. Open Questions

1. **Wrist F/T sensor model** — ATI Mini45, Robotiq FT 300-S, or OnRobot HEX-E?
2. **PLC model + protocol** — Modbus RTU, EtherNet/IP, or OPC-UA?
3. **DB engine** — SQLite (WAL) or PostgreSQL?
4. **Gemma 4 fine-tuning** — required for tool vocabulary, or sufficient with prompt engineering?
5. **Track C LLM model** — benchmark Qwen2.5 variants on Vector 16 HX GPU
6. **RL sim environment** — Isaac Sim vs MuJoCo for Doosan e0509?
7. **Safety E-stop v2.0 timeline** — when to add voice E-stop + vision safety zone?
8. **FOD alert threshold** — how long before "out" becomes "missing" (suggested: 30 min)?
