"""dashboard_node.py
────────────────────
FastAPI + WebSocket 대시보드 ROS2 노드.

카메라 4종 MJPEG 스트림 (ROS2 토픽 기반):
  /cam/gripper   ← /c270/image_raw          (그리퍼 원본)
  /cam/top       ← /d455f/color/image_raw   (탑뷰 원본)
  /cam/annotated ← /vision/debug/annotated  (YOLO 어노테이션)
  /cam/mask      ← /vision/debug/mask       (세그멘테이션 마스크)

검출 현황 실시간, 그리퍼 전류 파형,
fetch/return/home/E-stop 버튼, DB 상태 / 이벤트 로그 탭.

의존 (pip): fastapi uvicorn opencv-python-headless
"""

from __future__ import annotations

import asyncio
import json
import sqlite3
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image as ImgMsg, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from interfaces.msg import Intent, RobotStatus

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

# cv_bridge는 NumPy 1.x/2.x 충돌 위험이 있으므로 사용하지 않는다.
# ROS Image → numpy 변환은 _imgmsg_to_bgr()에서 직접 처리.

try:
    from vision_msgs.msg import Detection2DArray
    _VISION_OK = True
except ImportError:
    _VISION_OK = False

try:
    from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse, StreamingResponse
    import uvicorn
    _FASTAPI_OK = True
except ImportError:
    _FASTAPI_OK = False

_HOST = "0.0.0.0"
_PORT = 8080
_GRIPPER_CURRENT_MAXLEN = 200
_FRAME_STALE_SEC = 5.0   # 이 시간 이상 미수신 → stale

_CAM_TOPICS: dict[str, str] = {
    "gripper":           "/c270/image_raw",
    "top":               "/d455f/color/image_raw",
    "gripper_annotated": "/vision/debug/gripper/annotated",
    "gripper_mask":      "/vision/debug/gripper/mask",
    "gripper_marker":    "/vision/debug/gripper_marker",
    "top_annotated":     "/vision/debug/top_view/annotated",
    "top_mask":          "/vision/debug/top_view/mask",
}


def _make_placeholder() -> bytes:
    if not _CV2_OK:
        return b""
    import numpy as np
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.putText(img, "NO SIGNAL", (50, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (40, 80, 200), 2)
    _, buf = cv2.imencode(".jpg", img)
    return buf.tobytes()


class DashboardNode(Node):
    """대시보드 ROS2 노드 + FastAPI 웹서버."""

    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.declare_parameter("db_path", "~/robot_tools.db")
        self._db_path = Path(
            self.get_parameter("db_path").get_parameter_value().string_value
        ).expanduser()

        # ── 공유 상태 ────────────────────────────────────────────────────
        self._state: dict[str, Any] = {
            "is_moving": False,
            "plc_state": "unknown",
            "tool_status": "unknown",
            "tool_id": "socket_19mm",
            "gripper_current": 0,
            "gripper_position": 0,
            "detections": {"gripper": [], "top_view": []},
            "health": {"robot": "stale", "plc": "stale", "db": "stale"},
        }
        self._state_lock = threading.Lock()
        self._gripper_history: deque[float] = deque(maxlen=_GRIPPER_CURRENT_MAXLEN)
        self._ws_clients: list[WebSocket] = []
        self._ws_lock = threading.Lock()
        self._web_loop = None

        # ── 카메라 프레임 저장 ────────────────────────────────────────────
        _ph = _make_placeholder()
        self._frames: dict[str, bytes] = {k: _ph for k in _CAM_TOPICS}
        self._frame_times: dict[str, float] = {k: 0.0 for k in _CAM_TOPICS}
        self._frames_lock = threading.Lock()

        # ── ROS2 구독 — 카메라 이미지 ────────────────────────────────────
        for name, topic in _CAM_TOPICS.items():
            self.create_subscription(
                ImgMsg, topic,
                lambda msg, _n=name: self._on_image(msg, _n),
                qos_profile_sensor_data,
            )

        # ── ROS2 구독 — 시스템 상태 ───────────────────────────────────────
        qos_r  = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        qos_be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(RobotStatus, "/robot/status", self._on_robot_status, qos_r)
        self.create_subscription(String, "/plc/system_state", self._on_plc_state, qos_be)
        self.create_subscription(JointState, "/gripper/state", self._on_gripper_state, qos_be)

        # ── ROS2 구독 — 검출 (yolo_node는 BEST_EFFORT로 발행) ─────────────
        if _VISION_OK:
            _qos_det = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                Detection2DArray, "/vision/detections/gripper",
                lambda m: self._on_detections(m, "gripper"), _qos_det,
            )
            self.create_subscription(
                Detection2DArray, "/vision/detections/top_view",
                lambda m: self._on_detections(m, "top_view"), _qos_det,
            )

        # ── ROS2 발행자 / 서비스 클라이언트 ─────────────────────────────
        self._intent_pub = self.create_publisher(Intent, "/voice/intent", 1)
        self._plc_reset_pub = self.create_publisher(Bool, "/plc_reset", 1)
        self._home_cli        = self.create_client(Trigger, "/tool_action_server/home")
        self._estop_cli       = self.create_client(Trigger, "/tool_action_server/estop")
        self._estop_reset_cli = self.create_client(Trigger, "/tool_action_server/estop_reset")
        self._open_toolbox_cli  = self.create_client(Trigger, "/tool_action_server/open_toolbox")
        self._close_toolbox_cli = self.create_client(Trigger, "/tool_action_server/close_toolbox")
        self._open_toolbox_clis  = {
            1: self._open_toolbox_cli,
            2: self.create_client(Trigger, "/tool_action_server/open_toolbox_l2"),
        }
        self._close_toolbox_clis = {
            1: self._close_toolbox_cli,
            2: self.create_client(Trigger, "/tool_action_server/close_toolbox_l2"),
        }

        self._scan_states: dict[int, str] = {}   # layer_id → 상태 문자열
        self._scan_layer_running: int | None = None   # 현재 스캔 중인 layer
        self._prev_is_moving: bool = False
        self._config_dir = Path("config")

        # ── 타이머 ───────────────────────────────────────────────────────
        self.create_timer(2.0, self._poll_db)
        self.create_timer(0.1, self._broadcast_state)   # 10 Hz

        if _FASTAPI_OK:
            threading.Thread(target=self._start_web, daemon=True).start()
            self.get_logger().info(f"[Dashboard] http://{_HOST}:{_PORT}")
        else:
            self.get_logger().error("[Dashboard] fastapi/uvicorn 없음 — pip install fastapi uvicorn")

    # ── 카메라 콜백 ──────────────────────────────────────────────────────────

    @staticmethod
    def _imgmsg_to_bgr(msg: ImgMsg):
        """cv_bridge 없이 ROS Image → BGR numpy 배열 변환."""
        import numpy as np
        enc = msg.encoding.lower()
        if enc in ("bgr8", "rgb8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
            if enc == "rgb8":
                arr = arr[:, :, ::-1]
            return arr
        if enc == "mono8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if enc in ("16uc1",):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            norm = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            return cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
        return None

    def _on_image(self, msg: ImgMsg, name: str) -> None:
        if not _CV2_OK:
            return
        try:
            frame = self._imgmsg_to_bgr(msg)
            if frame is None:
                return
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
            with self._frames_lock:
                self._frames[name] = buf.tobytes()
                self._frame_times[name] = time.monotonic()
        except Exception as exc:
            self.get_logger().debug(f"[Dashboard] {name} 변환 실패: {exc}")

    # ── 검출 콜백 ────────────────────────────────────────────────────────────

    def _on_detections(self, msg: "Detection2DArray", camera_type: str) -> None:
        dets = []
        for d in msg.detections:
            if not d.results:
                continue
            h = d.results[0].hypothesis
            dets.append({
                "tool_id": h.class_id,
                "score":   round(float(h.score), 2),
                "cx":      round(d.bbox.center.position.x, 1),
                "cy":      round(d.bbox.center.position.y, 1),
                "w":       round(d.bbox.size_x, 1),
                "h":       round(d.bbox.size_y, 1),
            })
        with self._state_lock:
            self._state["detections"][camera_type] = dets

    # ── 시스템 상태 콜백 ──────────────────────────────────────────────────────

    def _on_robot_status(self, msg: RobotStatus) -> None:
        with self._state_lock:
            prev = self._prev_is_moving
            self._state["is_moving"] = msg.is_moving
            self._state["health"]["robot"] = "ok"
            self._prev_is_moving = msg.is_moving
        # is_moving True→False 전환: 스캔 중이었으면 완료로 표시
        if prev and not msg.is_moving and self._scan_layer_running is not None:
            layer = self._scan_layer_running
            self._scan_layer_running = None
            self._scan_states[layer] = "완료 — 결과 파일 확인"

    def _on_plc_state(self, msg: String) -> None:
        with self._state_lock:
            self._state["plc_state"] = msg.data
            self._state["health"]["plc"] = "ok"

    def _on_gripper_state(self, msg: JointState) -> None:
        if msg.effort:
            cur = float(msg.effort[0])
            with self._state_lock:
                self._state["gripper_current"] = cur
                self._gripper_history.append(cur)
        if msg.position:
            with self._state_lock:
                self._state["gripper_position"] = float(msg.position[0])

    def _poll_db(self) -> None:
        try:
            if not self._db_path.exists():
                return
            with sqlite3.connect(str(self._db_path), timeout=2) as conn:
                row = conn.execute(
                    "SELECT current_status FROM tools WHERE tool_id=?",
                    (self._state["tool_id"],),
                ).fetchone()
            with self._state_lock:
                self._state["tool_status"] = row[0] if row else "unknown"
                self._state["health"]["db"] = "ok"
        except Exception:
            with self._state_lock:
                self._state["health"]["db"] = "error"

    def _broadcast_state(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            snapshot = dict(self._state)
            snapshot["detections"] = {
                k: list(v) for k, v in self._state["detections"].items()
            }
            snapshot["gripper_history"] = list(self._gripper_history)
            with self._frames_lock:
                cam_health = {
                    name: "ok" if (now - self._frame_times[name]) < _FRAME_STALE_SEC
                    else "stale"
                    for name in _CAM_TOPICS
                }
            snapshot["health"] = {**snapshot["health"], **cam_health}

        if self._web_loop is None:
            return
        payload = json.dumps(snapshot)
        with self._ws_lock:
            clients = list(self._ws_clients)
        for ws in clients:
            fut = asyncio.run_coroutine_threadsafe(ws.send_text(payload), self._web_loop)

            def _on_done(f, _ws=ws):
                if f.exception() is not None:
                    with self._ws_lock:
                        try:
                            self._ws_clients.remove(_ws)
                        except ValueError:
                            pass

            fut.add_done_callback(_on_done)

    # ── 버튼 액션 ────────────────────────────────────────────────────────────

    def _pub_intent(self, intent_type: str, tool_id: str) -> None:
        msg = Intent()
        msg.intent_type = intent_type
        msg.tool_id = tool_id
        msg.raw_utterance = "dashboard"
        self._intent_pub.publish(msg)

    def _call_trigger(self, client, timeout: float = 60.0) -> dict:
        if not client.service_is_ready():
            return {"success": False, "message": "서비스 미준비 — tool_action_server 실행 확인"}
        req = Trigger.Request()
        future = client.call_async(req)
        done = threading.Event()
        result_holder: list = []
        future.add_done_callback(lambda f: (result_holder.append(f.result()), done.set()))
        done.wait(timeout=timeout)
        if not result_holder:
            return {"success": False, "message": f"타임아웃 ({timeout:.0f}s) — 동작 완료 후 재시도"}
        r = result_holder[0]
        return {"success": r.success, "message": r.message}

    # ── 운영/테스트 액션 ─────────────────────────────────────────────────────

    @staticmethod
    def _find_scan_script() -> str | None:
        """toolbox_scan_node.py 소스 경로를 탐색."""
        candidates = [
            Path("ros2_ws/src/motion/motion/toolbox_scan_node.py"),
            Path(__file__).parent.parent.parent.parent / "motion/motion/toolbox_scan_node.py",
        ]
        for p in candidates:
            if p.exists():
                return str(p)
        return None

    def _start_scan(self, layer_id: int) -> dict:
        """toolbox_scan_node를 백그라운드 서브프로세스로 기동."""
        if layer_id not in (1, 2):
            return {"success": False, "message": f"layer_id={layer_id} 는 1 또는 2만 허용"}
        proc = self._scan_procs.get(layer_id)
        if proc is not None and proc.poll() is None:
            return {"success": False, "message": f"{layer_id}층 스캔이 이미 실행 중입니다"}
        script = self._find_scan_script()
        if script is None:
            return {"success": False, "message": "toolbox_scan_node.py 를 찾을 수 없습니다"}
        try:
            p = subprocess.Popen(
                ["python3", script, "--ros-args", "-p", f"layer_id:={layer_id}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._scan_procs[layer_id] = p
            self.get_logger().info(f"[Dashboard] 스캔 시작 layer_id={layer_id} pid={p.pid}")
            return {"success": True, "message": f"{layer_id}층 스캔 시작 (pid={p.pid}). 완료 시 config/에 yaml 저장됩니다."}
        except Exception as e:
            return {"success": False, "message": str(e)}

    def _scan_status(self, layer_id: int) -> dict:
        status = self._scan_states.get(layer_id, "미실행")
        running = self._scan_layer_running == layer_id
        return {"running": running, "status": status}

    def _scan_results(self) -> list[dict]:
        """config/ 폴더의 toolbox_scan_*.yaml 파일 목록 반환."""
        files = sorted(self._config_dir.glob("toolbox_scan_*.yaml"), reverse=True)
        out = []
        for f in files:
            try:
                import yaml
                data = yaml.safe_load(f.read_text())
                out.append({"filename": f.name, "meta": data.get("scan_meta", {}),
                             "tools": data.get("tools", [])})
            except Exception:
                out.append({"filename": f.name, "meta": {}, "tools": []})
        return out

    def _db_reset_all(self) -> dict:
        """모든 공구 상태를 in_slot으로 초기화."""
        try:
            if not self._db_path.exists():
                return {"success": False, "message": "DB 파일 없음"}
            with sqlite3.connect(str(self._db_path), timeout=5) as conn:
                conn.execute("UPDATE tools SET current_status='in_slot', last_updated=strftime('%Y-%m-%dT%H:%M:%fZ','now')")
                conn.execute(
                    "INSERT INTO system_events(event_type, track, severity, notes) "
                    "VALUES ('boot','A','info','dashboard: 전체 공구 in_slot 초기화')"
                )
                conn.commit()
            return {"success": True, "message": "모든 공구 상태가 in_slot으로 초기화됐습니다."}
        except Exception as e:
            return {"success": False, "message": str(e)}

    def _db_set_tool_status(self, tool_id: str, status: str) -> dict:
        valid = {"in_slot", "out", "staged", "missing", "fod_alert"}
        if status not in valid:
            return {"success": False, "message": f"유효하지 않은 상태: {status}. 허용: {sorted(valid)}"}
        try:
            if not self._db_path.exists():
                return {"success": False, "message": "DB 파일 없음"}
            with sqlite3.connect(str(self._db_path), timeout=5) as conn:
                n = conn.execute(
                    "UPDATE tools SET current_status=?, last_updated=strftime('%Y-%m-%dT%H:%M:%fZ','now') WHERE tool_id=?",
                    (status, tool_id),
                ).rowcount
                conn.commit()
            if n == 0:
                return {"success": False, "message": f"tool_id={tool_id} 없음"}
            return {"success": True, "message": f"{tool_id} → {status}"}
        except Exception as e:
            return {"success": False, "message": str(e)}

    def _get_scan_pose_config(self) -> dict:
        import yaml
        cfg_path = "/home/kg/assistant/config/toolbox.yaml"
        try:
            with open(cfg_path, encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            gc = cfg.get("vision_motion", {}).get("gripper_cam_scan", {})
            return {
                "ok": True,
                "fetch_j_deg": gc.get("fetch_j_deg", []),
                "return_j_deg": gc.get("return_j_deg", []),
            }
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def _set_scan_pose_config(self, data: dict) -> dict:
        import yaml
        cfg_path = "/home/kg/assistant/config/toolbox.yaml"
        try:
            with open(cfg_path, encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            vm = cfg.setdefault("vision_motion", {})
            gc = vm.setdefault("gripper_cam_scan", {})
            if "fetch_j_deg" in data:
                gc["fetch_j_deg"] = [float(v) for v in data["fetch_j_deg"]]
            if "return_j_deg" in data:
                gc["return_j_deg"] = [float(v) for v in data["return_j_deg"]]
            with open(cfg_path, "w", encoding="utf-8") as f:
                yaml.dump(cfg, f, allow_unicode=True, default_flow_style=False)
            return {"ok": True, "message": "scan pose 저장 완료 — 다음 이동 명령부터 즉시 반영됩니다"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def _plc_reset(self) -> dict:
        msg = Bool()
        msg.data = True
        self._plc_reset_pub.publish(msg)
        return {"success": True, "message": "PLC 리셋 신호 전송 완료 (/plc_reset)"}

    def _get_ros2_nodes(self) -> list[str]:
        try:
            r = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True, text=True, timeout=5
            )
            return [n.strip() for n in r.stdout.strip().splitlines() if n.strip()]
        except Exception as e:
            return [f"오류: {e}"]

    # ── FastAPI 앱 ────────────────────────────────────────────────────────────

    def _start_web(self) -> None:
        app = self._build_app()
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._web_loop = loop
        config = uvicorn.Config(app, host=_HOST, port=_PORT,
                                loop="asyncio", log_level="warning")
        server = uvicorn.Server(config)
        server.install_signal_handlers = lambda: None
        loop.run_until_complete(server.serve())

    def _get_frame(self, name: str) -> bytes:
        with self._frames_lock:
            return self._frames.get(name, b"")

    def _build_app(self):
        from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
        from fastapi.responses import HTMLResponse, StreamingResponse

        app = FastAPI(title="Robot Demo Dashboard")

        try:
            static_dir = Path(get_package_share_directory("dashboard")) / "dashboard_static"
        except Exception:
            static_dir = Path(__file__).parent.parent / "dashboard_static"

        @app.get("/", response_class=HTMLResponse)
        async def index():
            p = static_dir / "index.html"
            return HTMLResponse(content=p.read_text() if p.exists()
                                else "<h1>index.html not found</h1>",
                                status_code=200 if p.exists() else 500)

        def _mjpeg_gen(name: str):
            while True:
                frame = self._get_frame(name)
                if frame:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                time.sleep(0.04)   # ~25 fps max

        @app.get("/cam/gripper")
        async def cam_gripper():
            return StreamingResponse(_mjpeg_gen("gripper"),
                                     media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/cam/top")
        async def cam_top():
            return StreamingResponse(_mjpeg_gen("top"),
                                     media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/cam/gripper_annotated")
        async def cam_gripper_annotated():
            return StreamingResponse(_mjpeg_gen("gripper_annotated"),
                                     media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/cam/gripper_mask")
        async def cam_gripper_mask():
            return StreamingResponse(_mjpeg_gen("gripper_mask"),
                                     media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/cam/gripper_marker")
        async def cam_gripper_marker():
            return StreamingResponse(_mjpeg_gen("gripper_marker"),
                                     media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/cam/top_annotated")
        async def cam_top_annotated():
            return StreamingResponse(_mjpeg_gen("top_annotated"),
                                     media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/cam/top_mask")
        async def cam_top_mask():
            return StreamingResponse(_mjpeg_gen("top_mask"),
                                     media_type="multipart/x-mixed-replace; boundary=frame")

        @app.websocket("/ws")
        async def ws_endpoint(ws: WebSocket):
            await ws.accept()
            with self._ws_lock:
                self._ws_clients.append(ws)
            try:
                while True:
                    await ws.receive_text()
            except WebSocketDisconnect:
                pass
            finally:
                with self._ws_lock:
                    try:
                        self._ws_clients.remove(ws)
                    except ValueError:
                        pass

        @app.post("/action/fetch")
        async def action_fetch(tool_id: str = "socket_19mm"):
            self._pub_intent("fetch", tool_id)
            return {"ok": True, "message": f"fetch {tool_id} 전송"}

        @app.post("/action/return")
        async def action_return(tool_id: str = "socket_19mm"):
            self._pub_intent("return", tool_id)
            return {"ok": True, "message": f"return {tool_id} 전송"}

        @app.post("/action/stage_pick_test")
        async def action_stage_pick_test(tool_id: str = "socket_19mm"):
            self._pub_intent("stage_pick_test", tool_id)
            return {"ok": True, "message": f"stage_pick_test {tool_id} 전송"}

        @app.post("/action/home")
        def action_home():
            self._pub_intent("home", "")
            return {"ok": True, "message": "home 명령 전송"}

        @app.post("/action/estop")
        def action_estop():
            return self._call_trigger(self._estop_cli, timeout=5.0)

        @app.post("/action/estop_reset")
        def action_estop_reset():
            return self._call_trigger(self._estop_reset_cli, timeout=5.0)

        @app.post("/action/open_toolbox")
        def action_open_toolbox(layer_id: int = 1):
            cli = self._open_toolbox_clis.get(layer_id, self._open_toolbox_cli)
            return self._call_trigger(cli, timeout=30.0)

        @app.post("/action/close_toolbox")
        def action_close_toolbox(layer_id: int = 1):
            cli = self._close_toolbox_clis.get(layer_id, self._close_toolbox_cli)
            return self._call_trigger(cli, timeout=30.0)

        @app.get("/api/db/tools")
        async def api_tools():
            try:
                if not self._db_path.exists():
                    return {"error": "DB 없음"}
                with sqlite3.connect(str(self._db_path), timeout=2) as conn:
                    conn.row_factory = sqlite3.Row
                    rows = conn.execute(
                        "SELECT tool_id, display_name, current_status, "
                        "home_slot_row, home_slot_col, last_updated FROM tools"
                    ).fetchall()
                return [dict(r) for r in rows]
            except Exception as e:
                return {"error": str(e)}

        @app.get("/api/db/events")
        async def api_events():
            try:
                if not self._db_path.exists():
                    return {"error": "DB 없음"}
                with sqlite3.connect(str(self._db_path), timeout=2) as conn:
                    conn.row_factory = sqlite3.Row
                    rows = conn.execute(
                        "SELECT tool_id, event_type, track, "
                        "status_before, status_after, notes, timestamp "
                        "FROM tool_events ORDER BY timestamp DESC LIMIT 100"
                    ).fetchall()
                return [dict(r) for r in rows]
            except Exception as e:
                return {"error": str(e)}

        # ── 운영/테스트 API ────────────────────────────────────────────────

        @app.post("/action/scan")
        def action_scan(layer_id: int = 1):
            self._scan_states[layer_id] = "실행 중"
            self._scan_layer_running = layer_id
            self._prev_is_moving = False  # 다음 True→False 전환을 정확히 감지하기 위해 초기화
            self._pub_intent("scan", str(layer_id))
            return {"ok": True, "message": f"{layer_id}층 스캔 명령 전송 (orchestrator BT 처리)"}

        @app.post("/action/scan_pose")
        async def action_scan_pose(request: Request):
            pose_type = request.query_params.get("type", "fetch")
            if pose_type not in ("fetch", "return"):
                pose_type = "fetch"
            self._pub_intent(f"scan_pose_{pose_type}", "")
            label = "공구 집기" if pose_type == "fetch" else "반납"
            return {"ok": True, "message": f"스캔 자세 이동 ({label}) 명령 전송"}

        @app.get("/api/config/scan_pose")
        def api_get_scan_pose():
            return self._get_scan_pose_config()

        @app.post("/api/config/scan_pose")
        async def api_set_scan_pose(request: Request):
            data = await request.json()
            return self._set_scan_pose_config(data)

        @app.get("/api/scan/status")
        def api_scan_status(layer_id: int = 1):
            return self._scan_status(layer_id)

        @app.get("/api/scan/results")
        def api_scan_results():
            return self._scan_results()

        @app.post("/action/db_reset_all")
        def action_db_reset_all():
            return self._db_reset_all()

        @app.post("/action/db_set_status")
        def action_db_set_status(tool_id: str, status: str):
            return self._db_set_tool_status(tool_id, status)

        @app.post("/action/plc_reset")
        def action_plc_reset():
            return self._plc_reset()

        @app.get("/api/system/nodes")
        def api_system_nodes():
            return {"nodes": self._get_ros2_nodes()}

        return app


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
