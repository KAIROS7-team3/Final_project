"""ScanLayer BT 서브트리 — scan_pose 이동 후 그리퍼캠 XY 수집·저장.

트리 구조:
    Sequence("ScanLayer_root")
    ├── SetMoving_true
    ├── RunAction_open_drawer   ← ExecutePhase(open_drawer, layer_id)
    ├── RunAction_scan_pose     ← ExecutePhase(scan_pose)
    ├── CollectAndSave          ← N초 대기하며 pose_buf 수집 후 yaml 저장
    ├── RunAction_home          ← ExecutePhase(home)
    ├── RunAction_close_drawer  ← ExecutePhase(close_drawer, layer_id)
    └── SetMoving_false
"""
from __future__ import annotations

import os
import time
import threading
import yaml
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import py_trees

from interfaces.action import ExecutePhase
from orchestrator.bt_nodes.fault_handler import FaultHandlerNode
from orchestrator.bt_nodes.run_action import RunAction
from orchestrator.bt_nodes.set_moving import SetMoving
from orchestrator.blackboard import KEY_ACTIVE_TOOL_ID

_CONFIG_DIR = Path("config")
_MIN_SAMPLES = 3
_COLLECT_SEC = 5.0


class CollectAndSave(py_trees.behaviour.Behaviour):
    """스캔 자세 유지 중 그리퍼캠 XY 데이터를 수집하고 yaml로 저장하는 BT 리프.

    Args:
        pose_buf: orchestrator_node이 /vision/tool_gripper_pose 콜백에서 채우는 공유 리스트.
                  요소: {"tool_id": str, "x": float, "y": float}
        buf_lock: pose_buf 보호용 threading.Lock.
        layer_id: 스캔 층 번호 (파일명에 포함).
        collect_sec: 수집 시간 (초). 기본 5.0.
    """

    def __init__(
        self,
        name: str,
        pose_buf: list,
        buf_lock: threading.Lock,
        layer_id: int,
        collect_sec: float = _COLLECT_SEC,
    ) -> None:
        super().__init__(name=name)
        self._pose_buf = pose_buf
        self._buf_lock = buf_lock
        self._layer_id = layer_id
        self._collect_sec = collect_sec

    def initialise(self) -> None:
        with self._buf_lock:
            self._pose_buf.clear()

    def update(self) -> py_trees.common.Status:
        self.logger.info(f"[CollectAndSave] {self._collect_sec}초 수집 시작 (layer={self._layer_id})")
        time.sleep(self._collect_sec)

        with self._buf_lock:
            buf_copy = list(self._pose_buf)

        if not buf_copy:
            self.logger.warning("[CollectAndSave] 수집된 데이터 없음 — vision 노드 확인 필요")
            return py_trees.common.Status.FAILURE

        result = self._aggregate(buf_copy)
        if not result:
            self.logger.warning("[CollectAndSave] 유효 샘플 부족 — 스캔 실패")
            return py_trees.common.Status.FAILURE

        self._save(result)
        return py_trees.common.Status.SUCCESS

    def _aggregate(self, buf: list) -> dict[str, dict]:
        groups: dict[str, list] = defaultdict(list)
        for item in buf:
            groups[item["tool_id"]].append((item["x"], item["y"]))

        out = {}
        for tool_id, samples in groups.items():
            if len(samples) < _MIN_SAMPLES:
                self.logger.warning(
                    f"[CollectAndSave] {tool_id}: 샘플 부족 ({len(samples)} < {_MIN_SAMPLES}) — 제외"
                )
                continue
            avg_x = sum(s[0] for s in samples) / len(samples)
            avg_y = sum(s[1] for s in samples) / len(samples)
            out[tool_id] = {"x": round(avg_x, 5), "y": round(avg_y, 5), "samples": len(samples)}
            self.logger.info(
                f"[CollectAndSave] {tool_id}: x={avg_x*1000:.1f}mm y={avg_y*1000:.1f}mm [n={len(samples)}]"
            )
        return out

    def _save(self, result: dict) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = _CONFIG_DIR / f"toolbox_scan_{ts}_layer{self._layer_id}.yaml"
        data = {
            "scan_meta": {
                "layer_id": self._layer_id,
                "timestamp": ts,
                "collect_sec": self._collect_sec,
                "note": "grasp_pose_base.x/y 에 수동 반영 후 커밋",
            },
            "tools": [
                {
                    "tool_id": tid,
                    "grasp_pose_base": {"x": v["x"], "y": v["y"]},
                    "samples": v["samples"],
                }
                for tid, v in result.items()
            ],
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
        self.logger.info(f"[CollectAndSave] 저장 완료: {path}")
        self.logger.info("[CollectAndSave] ⚠️  toolbox.yaml grasp_pose_base 수동 반영 후 커밋")


def build_scan_subtree(
    execute_phase_client: Any,
    pose_buf: list,
    buf_lock: threading.Lock,
    publish_status_fn: Callable[[bool], None],
    set_plc_fn: Callable[[str], None],
    log_error_fn: Callable[[str, str], None],
    layer_id: int = 1,
    collect_sec: float = _COLLECT_SEC,
) -> py_trees.behaviour.Behaviour:
    """ScanLayer 서브트리를 조립해 루트 노드를 반환한다."""

    def _open_goal(_tool_id: str) -> ExecutePhase.Goal:
        g = ExecutePhase.Goal()
        g.phase = "open_drawer"
        g.tool_id = ""
        g.layer_id = layer_id
        return g

    def _scan_goal(_tool_id: str) -> ExecutePhase.Goal:
        g = ExecutePhase.Goal()
        g.phase = "scan_pose"
        g.tool_id = ""
        g.layer_id = 0
        return g

    def _home_goal(_tool_id: str) -> ExecutePhase.Goal:
        g = ExecutePhase.Goal()
        g.phase = "home"
        g.tool_id = ""
        g.layer_id = 0
        return g

    def _close_goal(_tool_id: str) -> ExecutePhase.Goal:
        g = ExecutePhase.Goal()
        g.phase = "close_drawer"
        g.tool_id = ""
        g.layer_id = layer_id
        return g

    main_seq = py_trees.composites.Sequence(f"ScanLayer_main_l{layer_id}", memory=True)
    main_seq.add_children([
        SetMoving(
            "SetMoving_true",
            publish_fn=publish_status_fn,
            is_moving=True,
            set_plc_fn=set_plc_fn,
            plc_state="moving",
        ),
        RunAction("RunAction_open_drawer", execute_phase_client, _open_goal, timeout_sec=60.0),
        RunAction("RunAction_scan_pose",   execute_phase_client, _scan_goal,  timeout_sec=30.0),
        CollectAndSave(
            name="CollectAndSave",
            pose_buf=pose_buf,
            buf_lock=buf_lock,
            layer_id=layer_id,
            collect_sec=collect_sec,
        ),
        RunAction("RunAction_home",          execute_phase_client, _home_goal,  timeout_sec=60.0),
        RunAction("RunAction_close_drawer",  execute_phase_client, _close_goal, timeout_sec=60.0),
        SetMoving(
            "SetMoving_false",
            publish_fn=publish_status_fn,
            is_moving=False,
            set_plc_fn=set_plc_fn,
            plc_state="idle",
        ),
    ])

    motion_selector = py_trees.composites.Selector(f"ScanLayer_motion_l{layer_id}", memory=False)
    motion_selector.add_children([
        main_seq,
        FaultHandlerNode(
            name="FaultHandler",
            execute_phase_client=execute_phase_client,
            publish_status_fn=publish_status_fn,
            set_plc_fn=set_plc_fn,
            log_error_fn=log_error_fn,
            layer_id=layer_id,
        ),
    ])

    root = py_trees.composites.Sequence(f"ScanLayer_root_l{layer_id}", memory=True)
    root.add_children([motion_selector])
    return root
