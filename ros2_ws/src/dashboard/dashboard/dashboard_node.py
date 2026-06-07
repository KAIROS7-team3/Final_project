"""dashboard_node.py
───────────────────
FastAPI + WebSocket 대시보드 ROS2 노드.

카메라 2대 MJPEG 스트림, 그리퍼 전류 파형, fetch/return/home/E-stop 버튼,
DB 상태 / 이벤트 로그 탭. 위젯별 독립 health 표시 — 일부 소스 장애 시 페이지 유지.

의존 (pip): fastapi uvicorn opencv-python-headless
            (RealSense는 선택: pyrealsense2)
"""

from __future__ import annotations

import asyncio
import json
import os
import sqlite3
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from interfaces.msg import Intent, RobotStatus

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    import fastapi
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse, StreamingResponse
    import uvicorn
    _FASTAPI_OK = True
except ImportError:
    _FASTAPI_OK = False

_HOST = "0.0.0.0"
_PORT = 8080
_GRIPPER_CURRENT_MAXLEN = 200  # 파형 표시용 최근 N개


class CameraWorker:
    """독립 카메라 스레드 — 장애 시 placeholder 프레임 반환."""

    _PLACEHOLDER: bytes | None = None

    def __init__(self, device: str, name: str, fourcc: str = "YUYV") -> None:
        self.name = name
        self._device = device
        self._fourcc = fourcc
        self._frame: bytes | None = None
        self._lock = threading.Lock()
        self._health = "error"
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @classmethod
    def _get_placeholder(cls) -> bytes:
        if cls._PLACEHOLDER is None:
            if _CV2_OK:
                import numpy as np
                img = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(img, "CAM OFFLINE", (40, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 80, 200), 2)
                _, buf = cv2.imencode(".jpg", img)
                cls._PLACEHOLDER = buf.tobytes()
            else:
                cls._PLACEHOLDER = b""
        return cls._PLACEHOLDER

    def _run(self) -> None:
        while True:
            if not _CV2_OK:
                time.sleep(5)
                continue
            cap = cv2.VideoCapture(self._device)
            if not cap.isOpened():
                self._health = "error"
                time.sleep(3)
                continue
            if self._fourcc:
                cap.set(cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*self._fourcc))
            self._health = "ok"
            while True:
                ok, frame = cap.read()
                if not ok:
                    self._health = "error"
                    break
                _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                with self._lock:
                    self._frame = buf.tobytes()
            cap.release()
            time.sleep(2)

    def get_frame(self) -> bytes:
        with self._lock:
            return self._frame or self._get_placeholder()

    @property
    def health(self) -> str:
        return self._health


class DashboardNode(Node):
    """대시보드 ROS2 노드 + FastAPI 웹서버."""

    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.declare_parameter("db_path", "~/robot_tools.db")
        self.declare_parameter("gripper_cam", "/dev/gripper_cam")
        self.declare_parameter("top_cam", "/dev/top_cam")

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
            "health": {
                "robot": "stale",
                "plc": "stale",
                "db": "stale",
                "gripper_cam": "stale",
                "top_cam": "stale",
            },
        }
        self._state_lock = threading.Lock()
        self._gripper_history: deque[float] = deque(maxlen=_GRIPPER_CURRENT_MAXLEN)
        self._ws_clients: list[WebSocket] = []
        self._ws_lock = threading.Lock()
        self._web_loop = None  # FastAPI asyncio 루프 참조 (브로드캐스트용)

        # ── ROS2 구독 ────────────────────────────────────────────────────
        qos_r = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        qos_be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._robot_sub = self.create_subscription(
            RobotStatus, "/robot/status", self._on_robot_status, qos_r
        )
        self._plc_sub = self.create_subscription(
            String, "/plc/system_state", self._on_plc_state, qos_be
        )
        self._gripper_sub = self.create_subscription(
            JointState, "/gripper/state", self._on_gripper_state, qos_be
        )

        # ── ROS2 발행자 / 클라이언트 ─────────────────────────────────────
        self._intent_pub = self.create_publisher(Intent, "/voice/intent", 1)
        self._home_cli = self.create_client(Trigger, "/tool_action_server/home")
        self._estop_cli = self.create_client(Trigger, "/tool_action_server/estop")
        self._estop_reset_cli = self.create_client(
            Trigger, "/tool_action_server/estop_reset"
        )

        # ── 카메라 워커 ───────────────────────────────────────────────────
        gripper_cam_dev = (
            self.get_parameter("gripper_cam").get_parameter_value().string_value
        )
        top_cam_dev = self.get_parameter("top_cam").get_parameter_value().string_value
        self._cam_gripper = CameraWorker(gripper_cam_dev, "gripper_cam")
        self._cam_top = CameraWorker(top_cam_dev, "top_cam", fourcc="")

        # ── DB 폴링 타이머 ────────────────────────────────────────────────
        self.create_timer(2.0, self._poll_db)

        # ── WebSocket 브로드캐스트 타이머 ─────────────────────────────────
        self.create_timer(0.2, self._broadcast_state)

        # ── FastAPI 서버 스레드 ───────────────────────────────────────────
        if _FASTAPI_OK:
            threading.Thread(target=self._start_web, daemon=True).start()
            self.get_logger().info(
                f"[Dashboard] 웹서버 http://{_HOST}:{_PORT}"
            )
        else:
            self.get_logger().error(
                "[Dashboard] fastapi/uvicorn 없음 — pip install fastapi uvicorn"
            )

    # ── ROS2 콜백 ────────────────────────────────────────────────────────────

    def _on_robot_status(self, msg: RobotStatus) -> None:
        with self._state_lock:
            self._state["is_moving"] = msg.is_moving
            self._state["health"]["robot"] = "ok"

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
        """WebSocket 클라이언트에 현재 상태를 JSON으로 브로드캐스트."""
        with self._state_lock:
            snapshot = dict(self._state)
            snapshot["gripper_history"] = list(self._gripper_history)
            snapshot["health"]["gripper_cam"] = self._cam_gripper.health
            snapshot["health"]["top_cam"] = self._cam_top.health

        payload = json.dumps(snapshot)
        dead: list[WebSocket] = []
        with self._ws_lock:
            clients = list(self._ws_clients)
        if self._web_loop is None:
            return
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

    def _pub_intent(self, intent_type: str, tool_id: str = "socket_19mm") -> None:
        msg = Intent()
        msg.intent_type = intent_type
        msg.tool_id = tool_id
        msg.raw_utterance = "dashboard"
        self._intent_pub.publish(msg)

    def _call_trigger(self, client) -> dict:
        if not client.service_is_ready():
            return {"success": False, "message": "서비스 미준비"}
        req = Trigger.Request()
        future = client.call_async(req)
        done = threading.Event()
        result_holder: list = []
        future.add_done_callback(lambda f: (result_holder.append(f.result()), done.set()))
        done.wait(timeout=5.0)
        if not result_holder:
            return {"success": False, "message": "타임아웃"}
        r = result_holder[0]
        return {"success": r.success, "message": r.message}

    # ── FastAPI 웹서버 ────────────────────────────────────────────────────────

    def _start_web(self) -> None:
        app = self._build_app()
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._web_loop = loop  # ROS2 타이머에서 run_coroutine_threadsafe로 접근
        config = uvicorn.Config(app, host=_HOST, port=_PORT,
                                loop="asyncio", log_level="warning")
        server = uvicorn.Server(config)
        loop.run_until_complete(server.serve())

    def _build_app(self) -> "FastAPI":
        from fastapi import FastAPI, WebSocket, WebSocketDisconnect
        from fastapi.responses import HTMLResponse, StreamingResponse

        app = FastAPI(title="Robot Demo Dashboard")

        static_dir = Path(__file__).parent.parent / "dashboard_static"

        @app.get("/", response_class=HTMLResponse)
        async def index():
            p = static_dir / "index.html"
            if p.exists():
                return HTMLResponse(content=p.read_text())
            return HTMLResponse("<h1>index.html not found</h1>", status_code=500)

        def _mjpeg_gen(cam: CameraWorker):
            while True:
                frame = cam.get_frame()
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + frame + b"\r\n"
                )
                time.sleep(0.05)

        @app.get("/cam/gripper")
        async def cam_gripper():
            return StreamingResponse(
                _mjpeg_gen(self._cam_gripper),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        @app.get("/cam/top")
        async def cam_top():
            return StreamingResponse(
                _mjpeg_gen(self._cam_top),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

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
        async def action_fetch():
            self._pub_intent("fetch")
            return {"ok": True}

        @app.post("/action/return")
        async def action_return():
            self._pub_intent("return")
            return {"ok": True}

        @app.post("/action/home")
        async def action_home():
            return self._call_trigger(self._home_cli)

        @app.post("/action/estop")
        async def action_estop():
            return self._call_trigger(self._estop_cli)

        @app.post("/action/estop_reset")
        async def action_estop_reset():
            return self._call_trigger(self._estop_reset_cli)

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
                        "SELECT * FROM tool_events ORDER BY timestamp DESC LIMIT 100"
                    ).fetchall()
                return [dict(r) for r in rows]
            except Exception as e:
                return {"error": str(e)}

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
