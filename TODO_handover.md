# handover 브랜치 내일 확인 목록

## 운영 제약 (확인된 한계)

- ⚠️ **손 Z 허용 상한: 17cm (0.17m)**
  - 손 높이가 17cm 초과 시 pre_approach Z가 높아져 DSR 컨트롤러 알람(OnLogAlarm level:1) → 연결 끊김
  - `config/handover.yaml` `hand_z_max_m: 0.17` 참조
  - 코드에서 hand_z > hand_z_max_m 이면 abort 처리 필요 (미구현)

## 코드 수정 필요

- [ ] Force 모니터링 미구현 (S-6) — DSR joint torque 구독 추가 필요
      handover.yaml `force_torque_abort_nm: 5.0` 정의만 있고 코드에서 미사용

## 문서 보완 필요

- [ ] `docs/interfaces.md` §3에 PlaceOnHand.action 항목 추가
- [ ] `docs/interfaces.md` §4에 `/hand/pose`, `/hand/ready` 토픽 행 추가

## 비전팀 확인 요청

- [ ] `gripper_marker_scan_node.py`가 `PoseStamped`로 발행 중인지 확인
      (`PointStamped` → `PoseStamped` 변경 완료 여부, motion/README.md PR 검토 필수 주석)

## 실물 테스트 절차

1. 핸드오버 시퀀스 시작 → ① 손 없이 → 즉시 abort 확인 (staging fallback)
2. ① 통과 후 ⑧ GRIP_TOOL 성공 → ⑪에서 손 치우기 → 로봇 정지 확인
   - 로그: `[TAS] 공구 파지 후 abort — 모션 정지, 운영자 확인 필요`
   - PLC 빨강 점멸 확인
   - STT 차단 유지 확인
3. estop_reset 후 is_moving=False 발행 → STT 재개 확인
4. ⑪ 손 이동 3cm 이상 → abort 확인
   - 로그: `[TAS] 손 이동 XX.Xmm > 30mm — abort (S-6)`
