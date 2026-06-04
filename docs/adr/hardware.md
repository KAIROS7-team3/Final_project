# ADR — 하드웨어

> 참조: [인덱스](index.md)
> 하드웨어 인벤토리·드라이버·네트워크 → [`docs/hardware.md`](../hardware.md)

---

## ADR-009: PLC 프로토콜 — Modbus RTU (RS-485)

- **하드웨어**: LS Electric XBC-DR14E (결정 #2)
- **결정**: Modbus RTU via RS-485. `/dev/plc` (udev 심링크), `pymodbus` 라이브러리
- **이유**: 별도 Ethernet 포트 불필요, 단일 RS-485 케이블로 배선 단순화, 저속 LED 제어에 충분한 대역폭
- **제약**: 통신 속도 기본 9600 baud (XBC-DR14E 기본값). Phase 1 bring-up 시 VID/PID + baud rate 확인 필요

---

## ADR-010: Doosan 제어 인터페이스 — 트랙별 분리

| 트랙 | API | 특징 |
|------|-----|------|
| Track A/B | `doosan-robot2` ROS2 드라이버 | ROS2 topic/action 기반, 안전 기능 내장 |
| Track C | Doosan Python SDK | ROS2 우회, 직접 TCP/IP 통신 |

- 동일 joint limit / workspace limit이 양 트랙에 적용되어야 함
- Track C SDK 직접 제어 시 Safety Controller 연동 확인 필수 (ADR-005)
