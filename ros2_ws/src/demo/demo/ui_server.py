"""Demo UI 서버 — FastAPI + ROS2 브리지.

ROS2 토픽을 WebSocket으로 브라우저에 실시간 스트리밍한다.

구독 토픽:
  /voice/raw_text   (std_msgs/String)          → 음성 텍스트
  /voice/intent     (interfaces/Intent)         → 파싱된 intent
  /plc/status       (interfaces/PLCStatus)      → PLC LED 상태
  /gripper/state    (sensor_msgs/JointState)    → 그리퍼 전류(effort[0])

DB: db_core를 2초 주기로 직접 폴링.

의존성 (1회 설치):
    /usr/bin/pip3 install --user fastapi "uvicorn[standard]" opencv-python-headless

실행:
    ros2 run demo demo_ui
    브라우저: http://localhost:8765
"""

from __future__ import annotations

import asyncio
import os
import sqlite3
import sys
import threading
import time
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Any

# ── 선택적 의존성 ────────────────────────────────────────────────────────────

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
    _FASTAPI_OK = True
except ImportError:
    _FASTAPI_OK = False
    FastAPI = WebSocket = None  # type: ignore[misc,assignment]

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    import pyrealsense2 as rs
    import numpy as np
    _RS_OK = True
except ImportError:
    _RS_OK = False

# ── ROS2 ─────────────────────────────────────────────────────────────────────

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    _ROS2_OK = True
except ImportError:
    _ROS2_OK = False
    Node = object  # type: ignore[misc,assignment]

# ── 레포 루트를 sys.path에 추가 ──────────────────────────────────────────────

def _add_repo_root() -> Path:
    node = Path(__file__).resolve()
    while node != node.parent:
        if (node / "unit_actions").is_dir():
            if str(node) not in sys.path:
                sys.path.insert(0, str(node))
            return node
        node = node.parent
    return Path(__file__).resolve().parents[5]

_REPO_ROOT = _add_repo_root()

# ── 전역 상태 ────────────────────────────────────────────────────────────────

_UI_PORT = 8765
_DB_POLL_SEC = 2.0
_CAM_FPS = 20

_loop: asyncio.AbstractEventLoop | None = None
_event_queue: asyncio.Queue | None = None
_ws_clients: set[WebSocket] = set()
_bridge_node: UIBridgeNode | None = None  # type: ignore[name-defined]  # defined below

_tool_id: str = "socket_19mm"
_db_path: str = os.path.expanduser("~/robot_tools.db")

# Camera: 별도 스레드에서 갱신, MJPEG 엔드포인트에서 읽음
_latest_frames: dict[int, bytes | None] = {2: None, 8: None}  # C270=2, RealSense color=8


# ── 설정 로드 ─────────────────────────────────────────────────────────────────

def _load_demo_config() -> None:
    global _tool_id, _db_path
    try:
        import yaml
        cfg_path = _REPO_ROOT / "config" / "demo.yaml"
        with open(cfg_path) as f:
            cfg = (yaml.safe_load(f) or {}).get("demo", {})
        _tool_id = cfg.get("tool_id", _tool_id)
        _db_path = os.path.expanduser(cfg.get("db_path", _db_path))
    except Exception:
        pass


# ── ROS2 브리지 노드 ──────────────────────────────────────────────────────────

class UIBridgeNode(Node):  # type: ignore[misc]
    """ROS2 토픽 → WebSocket 이벤트 변환 노드."""

    def __init__(self) -> None:
        super().__init__("demo_ui_node")

        # /voice/raw_text
        self.create_subscription(String, "/voice/raw_text", self._on_raw_text, 10)

        # /voice/intent
        try:
            from interfaces.msg import Intent
            self.create_subscription(Intent, "/voice/intent", self._on_intent, 1)
            self._intent_pub = self.create_publisher(Intent, "/voice/intent", 1)
            self._Intent = Intent
        except Exception:
            self._intent_pub = None
            self._Intent = None

        # /plc/status
        try:
            from interfaces.msg import PLCStatus
            self.create_subscription(PLCStatus, "/plc/status", self._on_plc, 10)
        except Exception:
            pass

        # /gripper/state (effort[0] = current mA)
        self.create_subscription(
            JointState, "/gripper/state", self._on_gripper, qos_profile_sensor_data
        )

        self.get_logger().info(f"[ui] 대시보드: http://localhost:{_UI_PORT}")

    # ── 트리거 ─────────────────────────────────────────────────────────────────

    def publish_intent(self, intent_type: str) -> None:
        if self._intent_pub is None or self._Intent is None:
            return
        msg = self._Intent()
        msg.intent_type = intent_type
        msg.tool_id = _tool_id
        msg.confidence = 1.0
        msg.raw_utterance = f"[UI] {intent_type} {_tool_id}"
        msg.timestamp = self.get_clock().now().to_msg()
        self._intent_pub.publish(msg)
        self._emit("log", {"level": "info", "msg": f"[UI 트리거] {intent_type} {_tool_id}"})

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _on_raw_text(self, msg: String) -> None:
        self._emit("voice_text", {"text": msg.data})

    def _on_intent(self, msg: Any) -> None:
        self._emit("intent", {
            "intent_type": msg.intent_type,
            "tool_id": msg.tool_id,
            "confidence": round(float(msg.confidence), 2),
        })

    def _on_plc(self, msg: Any) -> None:
        self._emit("plc_status", {
            "state": msg.system_state,
            "led_color": msg.led_color,
            "led_mode": msg.led_mode,
        })

    def _on_gripper(self, msg: JointState) -> None:
        current_ma = int(msg.effort[0]) if msg.effort else 0
        position = int(msg.position[0]) if msg.position else 0
        self._emit("gripper", {"current_ma": current_ma, "position": position})

    # ── 이벤트 emit ───────────────────────────────────────────────────────────

    def _emit(self, event_type: str, data: dict) -> None:
        if _loop is None or _event_queue is None:
            return
        payload = {"type": event_type, "ts": _ts(), **data}
        _loop.call_soon_threadsafe(_event_queue.put_nowait, payload)


# ── DB 폴링 ───────────────────────────────────────────────────────────────────

def _query_db() -> dict:
    try:
        conn = sqlite3.connect(_db_path, timeout=3.0)
        tools = conn.execute(
            "SELECT tool_id, current_status, last_updated FROM tools ORDER BY tool_id"
        ).fetchall()
        events = conn.execute(
            "SELECT tool_id, event_type, track, status_before, status_after, timestamp "
            "FROM tool_events ORDER BY timestamp DESC LIMIT 20"
        ).fetchall()
        conn.close()
        return {
            "tools": [
                {"tool_id": r[0], "status": r[1], "updated": (r[2] or "")[:19]}
                for r in tools
            ],
            "events": [
                {
                    "tool_id": r[0], "event": r[1], "track": r[2] or "",
                    "before": r[3] or "", "after": r[4] or "",
                    "ts": (r[5] or "")[:19],
                }
                for r in events
            ],
        }
    except Exception as exc:
        return {"error": str(exc), "tools": [], "events": []}


# ── 카메라 스레드 ─────────────────────────────────────────────────────────────

def _camera_worker(cam_idx: int) -> None:
    if not _CV2_OK:
        return
    cap = cv2.VideoCapture(f"/dev/video{cam_idx}", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print(f"[ui] CAM {cam_idx} 열기 실패")
        return
    interval = 1.0 / _CAM_FPS
    while True:
        ret, frame = cap.read()
        if ret:
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            _latest_frames[cam_idx] = buf.tobytes()
        time.sleep(interval)


def _realsense_worker() -> None:
    if not _RS_OK or not _CV2_OK:
        return
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    try:
        pipeline.start(cfg)
    except Exception as exc:
        print(f"[ui] RealSense 시작 실패: {exc}")
        return
    interval = 1.0 / _CAM_FPS
    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            if color:
                img = np.asanyarray(color.get_data())
                _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 65])
                _latest_frames[8] = buf.tobytes()
            time.sleep(interval)
    except Exception as exc:
        print(f"[ui] RealSense 오류: {exc}")
    finally:
        pipeline.stop()


# ── FastAPI ───────────────────────────────────────────────────────────────────

@asynccontextmanager
async def _lifespan(app: FastAPI):  # type: ignore[type-arg]
    asyncio.create_task(_event_broadcaster())
    asyncio.create_task(_db_poller())
    yield


app = FastAPI(lifespan=_lifespan) if _FASTAPI_OK else None  # type: ignore[call-arg]


async def _event_broadcaster() -> None:
    while _event_queue is None:
        await asyncio.sleep(0.05)
    while True:
        event = await _event_queue.get()
        dead: set[WebSocket] = set()
        for ws in list(_ws_clients):
            try:
                await ws.send_json(event)
            except Exception:
                dead.add(ws)
        _ws_clients.difference_update(dead)


async def _db_poller() -> None:
    while True:
        await asyncio.sleep(_DB_POLL_SEC)
        data = await asyncio.get_running_loop().run_in_executor(None, _query_db)
        event = {"type": "db_state", "ts": _ts(), **data}
        for ws in list(_ws_clients):
            try:
                await ws.send_json(event)
            except Exception:
                pass


@app.get("/", response_class=HTMLResponse)  # type: ignore[union-attr]
async def dashboard():
    html = Path(__file__).parent / "ui_static" / "index.html"
    return html.read_text()


@app.websocket("/ws")  # type: ignore[union-attr]
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    _ws_clients.add(ws)
    data = await asyncio.get_running_loop().run_in_executor(None, _query_db)
    await ws.send_json({"type": "db_state", "ts": _ts(), **data})
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(ws)


@app.post("/trigger/{intent_type}")  # type: ignore[union-attr]
async def trigger(intent_type: str) -> JSONResponse:
    if intent_type not in ("fetch", "return"):
        return JSONResponse({"ok": False, "error": "fetch|return만 허용"}, status_code=400)
    if _bridge_node is None:
        return JSONResponse({"ok": False, "error": "ROS2 브리지 미연결"}, status_code=503)
    _bridge_node.publish_intent(intent_type)
    return JSONResponse({"ok": True})


@app.get("/camera/{cam_idx}", response_model=None)  # type: ignore[union-attr]
async def camera(cam_idx: int = 0):
    if cam_idx not in _latest_frames:
        return HTMLResponse("<p style='color:gray'>유효한 카메라 인덱스: 0 또는 1</p>", status_code=400)
    if not _CV2_OK:
        return HTMLResponse("<p style='color:gray'>opencv-python-headless 미설치</p>")

    async def _generate():
        while True:
            frame = _latest_frames.get(cam_idx)
            if frame:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            await asyncio.sleep(1.0 / _CAM_FPS)

    return StreamingResponse(
        _generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# ── ROS2 스레드 ───────────────────────────────────────────────────────────────

def _ros2_thread() -> None:
    global _bridge_node
    if not _ROS2_OK:
        print("[ui] rclpy 없음 — ROS2 없이 UI만 실행")
        return
    try:
        rclpy.init()
        _bridge_node = UIBridgeNode()
        executor = MultiThreadedExecutor()
        executor.add_node(_bridge_node)
        executor.spin()
    except Exception as exc:
        print(f"[ui] ROS2 오류: {exc}")
    finally:
        if _bridge_node is not None:
            _bridge_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ── 헬퍼 ─────────────────────────────────────────────────────────────────────

def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


# ── main ─────────────────────────────────────────────────────────────────────

async def _main_async() -> None:
    global _loop, _event_queue
    _loop = asyncio.get_running_loop()
    _event_queue = asyncio.Queue()

    _load_demo_config()

    threading.Thread(target=_ros2_thread, daemon=True).start()
    if _CV2_OK:
        threading.Thread(target=_camera_worker, args=(2,), daemon=True).start()  # C270
    threading.Thread(target=_realsense_worker, daemon=True).start()  # RealSense

    config = uvicorn.Config(app, host="0.0.0.0", port=_UI_PORT, log_level="warning")
    server = uvicorn.Server(config)
    server.install_signal_handlers = False
    print(f"[ui] 브라우저에서 열기: http://localhost:{_UI_PORT}")
    await server.serve()


def main(args: list[str] | None = None) -> None:
    if not _FASTAPI_OK:
        print(
            "ERROR: fastapi/uvicorn 미설치.\n"
            "  /usr/bin/pip3 install --user fastapi 'uvicorn[standard]' opencv-python-headless"
        )
        sys.exit(1)
    try:
        asyncio.run(_main_async())
    except KeyboardInterrupt:
        pass
