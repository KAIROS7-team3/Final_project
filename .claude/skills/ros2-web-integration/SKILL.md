---
name: ros2-web-integration
description: ROS2 노드를 웹 기술(REST, WebSocket, rosbridge, MJPEG, WebRTC)과 통합하는 패턴. 본 프로젝트 v1.0은 웹 UI를 사용하지 않음 — 운영자 인터페이스는 음성 + PLC LED. v2.0+ 원격 모니터링/대시보드 도입 검토 시에만 활성화. rosbridge, roslibjs, FastAPI with ROS2, Flask with rclpy, WebSocket for telemetry, MJPEG streaming, browser-based robot control, robot dashboard 시 트리거.
---

# ROS2 + 웹 통합 (v2.0+ 검토용)

> **본 프로젝트 v1.0은 웹 UI 미사용.** 운영자 인터페이스는 음성 명령(STT) + PLC LED 표시등 + (필요 시) 로컬 로그 뷰어로 완결된다.
> 전체 ROS2-웹 통합 패턴은 [`REFERENCE.md`](REFERENCE.md)에 보존.

---

## 왜 v1.0에서 웹을 안 쓰는가

| 이유 | 설명 |
|------|------|
| 운영 환경 | 실험실/소규모 정비소, 운영자 1명이 로봇 옆에 위치 |
| 안전 | 원격 명령은 E-stop 응답성·인지 부담 측면에서 v1.0 스코프 초과 |
| 복잡도 | 인증·CORS·WebSocket 관리·스트리밍 비용이 본 프로젝트 학습 목표와 무관 |
| 대안 | PLC LED(빨강/노랑/초록 점멸 패턴)가 핵심 상태를 충분히 전달 |

---

## v2.0+ 도입 검토 시 결정 사항

1. **목적 명확화** — 원격 모니터링? 원격 명령? 데이터 다운로드?
2. **인증** — 로컬망 한정인지, 인증 강도 (basic auth / OIDC / mTLS)
3. **안전 분리** — 모니터링 전용 vs 제어 가능. 제어는 별도 safety review 필요.
4. **스택 선택** — 아래 §스택 선택 가이드 참조

---

## 스택 선택 가이드 (v2.0+ 결정용)

| 요구사항 | 권장 스택 |
|---------|----------|
| ROS2 topic을 브라우저에 그대로 노출 | `rosbridge_suite` + `roslibjs` |
| REST API로 일부 동작 노출 | `FastAPI` + `rclpy` (별도 프로세스, ROS2 client) |
| 카메라 영상 스트리밍 | `web_video_server` (MJPEG) 또는 WebRTC (지연 중요 시) |
| 원격 텔레메트리 대시보드 | Grafana + InfluxDB (ROS2 → DB → Grafana) — 가장 단순 |
| 양방향 제어 | WebSocket + 인증 + 안전 검증 미들웨어 |

**원격 제어는 본 v1.0의 음성 명령 안전 모델([`../whisper-stt/SKILL.md`](../whisper-stt/SKILL.md), [`../../rules/safety.md`](../../rules/safety.md) S-2, S-7)을 우회할 위험이 있으므로 v2.0+에서도 별도 ADR 필요.**

---

## 빠른 참조 (v2.0+ POC용)

### rosbridge_suite (가장 흔한 선택)
```bash
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```
브라우저에서 `roslibjs`로 topic 구독·서비스 호출 가능.

### FastAPI + rclpy
별도 프로세스에서 `rclpy.init()` → ROS2 client 노드로 작동. CORS·인증은 FastAPI 미들웨어로.
```python
from fastapi import FastAPI
import rclpy
from rclpy.node import Node

rclpy.init()
app = FastAPI()
node = Node('web_bridge')

@app.get('/status')
def status():
    # ROS2 service call 또는 latest topic value
    ...
```

### 카메라 스트림
```bash
sudo apt install ros-humble-web-video-server
ros2 run web_video_server web_video_server
# http://<robot>:8080/stream?topic=/camera/color/image_raw
```

---

## 더 깊은 자료가 필요하면

- 전체 패턴(rosbridge·FastAPI·Flask·WebSocket·WebRTC·인증·rate limiting·async executor 충돌 등) → [`REFERENCE.md`](REFERENCE.md)
- rosbridge_suite: <https://github.com/RobotWebTools/rosbridge_suite>
- web_video_server: <https://github.com/RobotWebTools/web_video_server>
- roslibjs: <https://github.com/RobotWebTools/roslibjs>
- 원본 출처: <https://github.com/arpitg1304/robotics-agent-skills>
