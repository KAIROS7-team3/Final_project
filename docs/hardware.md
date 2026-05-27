# 하드웨어 인벤토리

> 드라이버 버전·캘리브레이션 파일 위치·알려진 한계를 기록한다.
> 하드웨어 변경(마운트, 펌웨어, 드라이버) 시 이 문서를 갱신하고 hand-eye 재캘리브레이션 여부를 검토한다.

---

## 1. 하드웨어 인벤토리

| 구분 | 모델 | 역할 | 상태 |
|------|------|------|------|
| 로봇팔 | Doosan Robotics e0509 | 협동 로봇, 5kg 페이로드, 900mm reach | ✅ 확정 |
| 그리퍼 | ROBOTIS RH-P12-RN | 이중 핑거 전기 그리퍼 | ✅ 확정 |
| 카메라 | Intel RealSense D455f | RGB-D, 스테레오 깊이, IMU | ✅ 확정 |
| F/T 센서 | 미사용 (v1.0) | — | ✅ 결정 #1 (미사용) |
| PLC | LS Electric XBC-DR10E | LED 슬롯 상태 표시 | ✅ 결정 #2 |
| 개발 머신 (주) | Vector 16 HX AI A2XWIG | GPU 추론, ROS2 실행 | ✅ 확정 |
| 개발 머신 (보조) | HP ProBook 450 G10 | 모니터링 대시보드 | ✅ 확정 |

---

## 2. 드라이버 및 패키지 버전

> 버전은 환경 구성 시 고정. 무단 업그레이드 금지 — 변경 시 이 표 갱신 필수.

| 컴포넌트 | 패키지 | 버전 (확정 후 기입) | 비고 |
|----------|--------|-------------------|------|
| ROS2 | `ros-humble-desktop` | Humble Hawksbill | Ubuntu 22.04 |
| Doosan 드라이버 | `doosan-robot2` | TBD | ROS2 Humble 브랜치 |
| RealSense 드라이버 | `realsense-ros` | TBD | D455f 지원 버전 |
| RH-P12-RN 드라이버 | Dynamixel SDK / 전용 패키지 | TBD | RS-485 통신 |
| PLC 라이브러리 | `pymodbus` | TBD | XBC-DR10E, Modbus RTU via RS-485 (ADR-009) |
| DDS | CycloneDDS | Humble 기본 | 단일 머신 설정 |

---

## 3. 하드웨어 한계 (알려진 값)

### Doosan e0509

| 항목 | 값 |
|------|-----|
| 페이로드 | 5kg |
| Reach | 900mm |
| 반복 정밀도 | ±0.03mm |
| 관절 속도 한계 | J1–J3: 180°/s, J4–J6: 225°/s |
| 협업 속도 한계 (ISO/TS 15066) | 250mm/s (운영자 접근 가능 공간) |
| 소프트 E-Stop 응답 | ≤ 500ms (.claude/rules/safety.md S-4) |

### Intel RealSense D455f

| 항목 | 값 |
|------|-----|
| RGB 해상도 | 1280×720 (작업 기본값) |
| 깊이 해상도 | 848×480 |
| 깊이 범위 | 0.6m – 6m |
| 깊이 FPS | 30fps |
| 실내 조명 | LED 보조 조명 권장 (YOLOv8 정확도 ≥ 95% 조건) |

### ROBOTIS RH-P12-RN

| 항목 | 값 |
|------|-----|
| 최대 파지력 | 170N |
| 파지 범위 | 0–109mm |
| 통신 | RS-485 (Dynamixel Protocol 2.0) |
| 제어 | gripper command 0~1 (0=open, 1=close) |

---

## 4. udev 규칙

장치 고정 경로 (Phase 1에서 설정):

```bash
# /etc/udev/rules.d/99-robot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="doosan"
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="gripper"
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="plc"
```

> Vendor/Product ID는 Phase 1 bring-up 시 `udevadm info` 로 확인 후 기입.

---

## 5. 캘리브레이션 파일 위치

| 파일 | 내용 | 갱신 조건 |
|------|------|----------|
| `config/hand_eye.yaml` | 카메라–로봇 변환 행렬 | 카메라 마운트 변경 또는 재투영 오차 > 1.5px |
| `config/robot_poses.yaml` | home, scan 관절 각도 | 작업 공간 레이아웃 변경 |
| `config/staging_area.yaml` | Staging Area 좌표 (base_link 기준) | 거치대 물리적 이동 |
| `config/toolbox.yaml` | 슬롯 좌표 + 공구 기하 | 공구함 레이아웃 변경 |

캘리브레이션 절차 상세 → [`.claude/skills/hand-eye-calibration/SKILL.md`](../.claude/skills/hand-eye-calibration/SKILL.md)

---

## 6. 개발 머신 환경 (Vector 16 HX)

| 항목 | 사양 |
|------|------|
| GPU | RTX 4090 Laptop (16GB VRAM) |
| VRAM 할당 | Track A: ~5.5–7.5GB / Track B: ~6.5–8.5GB / Track C: ~5–6GB(Q4) |
| CUDA | 확정 후 기입 |
| 컨테이너 | 미사용 (네이티브 Ubuntu 22.04 + ROS2 Humble) |

VRAM 실시간 확인:
```bash
nvidia-smi dmon -s mu -d 1
```

---

## 7. 네트워크 토폴로지

### 물리 연결 구성

```
Vector 16 HX (메인 PC)
├── [Ethernet] ──────── Doosan e0509 컨트롤러  (192.168.1.x 예정, TCP/IP)
├── [USB 3.x] ──────── RealSense D455f         (/dev/video*, eye-in-hand, 케이블 체인)
├── [USB-RS485] ─────── ROBOTIS RH-P12-RN      (/dev/gripper, Dynamixel Protocol 2.0)
├── [RS-485 Modbus RTU] ─ PLC XBC-DR10E        (/dev/plc, pymodbus)
└── [LAN / WiFi] ────── HP ProBook 450 G10     (모니터링 대시보드)
```

### 인터페이스 상세

| 장치 | 인터페이스 | 프로토콜 | 주소 / 포트 | udev 심링크 | 비고 |
|------|-----------|---------|------------|------------|------|
| Doosan e0509 | Ethernet (RJ45) | TCP/IP (Doosan SDK/ROS2) | `192.168.1.100` (예정) | — | Phase 0 ② 확정 |
| RealSense D455f | USB 3.x | UVC / libusb | `/dev/video*` | — | eye-in-hand, 케이블 체인 필요 |
| RH-P12-RN | USB → RS-485 | Dynamixel Protocol 2.0 | `/dev/gripper` | `SYMLINK+="gripper"` | Phase 1 bring-up 시 VID/PID 확인 |
| PLC (XBC-DR10E) | RS-485 | Modbus RTU | `/dev/plc` | `SYMLINK+="plc"` | Phase 1 bring-up 시 VID/PID + baud rate 확인 |
| HP ProBook | LAN / WiFi | HTTP (대시보드) | DHCP 또는 정적 | — | 모니터링 전용, 제어 없음 |

### IP 주소 정책

| 장치 | IP | 방식 |
|------|-----|------|
| Vector 16 HX (로봇망) | `192.168.1.10` (예정) | 정적 |
| Doosan e0509 컨트롤러 | `192.168.1.100` (예정) | 정적 |
| HP ProBook (모니터링) | DHCP | — |
| 외부 인터넷 | Vector 16 HX 별도 NIC | 개발망 분리 권장 |

> IP 정책은 Phase 0 네트워크 설정 시 확정. 로봇 전용 격리 서브넷(192.168.1.0/24) 권장.

### USB 포트 할당 권장

> 장치 충돌 방지를 위해 포트 번호를 고정하고 udev rules로 보완.

| USB 포트 | 장치 | 이유 |
|---------|------|------|
| USB 3.x #1 | RealSense D455f | 대역폭 우선 (RGB-D 스트림) |
| USB 2.0 #1 | RH-P12-RN (USB-RS485) | 저속 시리얼 충분 |
| USB 2.0 #2 | PLC XBC-DR10E (RS-485 경우) | 저속 시리얼 충분. Ethernet 선택 시 불필요 |
