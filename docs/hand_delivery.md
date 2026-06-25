# Hand-to-Hand 공구 전달 — Track A 적용 방안

## 개요

작업자가 손을 내밀면 로봇이 Staging Area 대신 **손바닥 위에 직접 공구를 올려놓는** 기능.
MediaPipe Hand Detection + RealSense D455F 깊이 카메라를 결합하여 3D 손 위치를 추적한다.

---

## 흐름 변경

```
기존:
  Robot → 공구 집기 → Staging Area에 내려놓기 → 작업자 픽업

변경:
  Robot → 공구 집기 → 작업자 손 감지 → 손바닥 위에 직접 올려놓기
```

Staging Area fallback 유지 → 손 감지 실패 시 기존 방식으로 동작.

---

## 핵심 기술 과제

### 1. Z값 문제 해결 (MediaPipe 한계)

MediaPipe 단독으로는 Z=0. RealSense D455F 깊이와 결합하여 해결한다.

```python
# vision/vision/hand_node.py (신규)

# MediaPipe → 2D 손바닥 중심 픽셀 좌표
palm_px = mediapipe_result.palm_center  # (u, v)

# RealSense 깊이 → Z 획득
depth_z = depth_frame.get_distance(palm_px.u, palm_px.v)

# 카메라 → 월드 좌표 변환 (hand_eye_loader 재사용)
hand_pos_world = hand_eye_transform @ [palm_px.u, palm_px.v, depth_z, 1]
```

기존 `vision/vision/hand_eye_loader.py` 좌표 변환 재사용 가능.

### 2. 손바닥 방향 감지

손바닥 법선 벡터를 계산하여 위를 향할 때만 전달 가능 상태로 판단.
손바닥 방향 → 공구 놓을 때 EE 자세 결정에 활용.

---

## 시스템 구조

### 신규 추가 파일

```
vision/vision/hand_node.py          ← MediaPipe + RealSense 깊이 융합
  발행 토픽:
    /hand/pose  (geometry_msgs/PoseStamped)  — 손바닥 3D 위치/자세
    /hand/ready (std_msgs/Bool)              — 손바닥 위 향함 + 안정적

unit_actions/place_on_hand.py       ← 동적 타겟 pose 기반 배치
  (place_at_staging.py 변형)

ros2_ws/src/orchestrator/orchestrator/bt_nodes/hand_delivery.py
  ← /hand/ready 구독 → 준비되면 place_on_hand 실행
```

### Blackboard 키 추가

```python
KEY_HAND_POSE  = "hand_pose"   # 손바닥 3D 위치/자세
KEY_HAND_READY = "hand_ready"  # 손바닥 위 향함 + 안정적
```

---

## BT 변경 포인트

```
기존 FetchTool 서브트리:
  ExecuteTask2 → PlaceAtStaging

변경 후:
  ExecuteTask2 → Selector(
      Sequence(CheckHandReady, PlaceOnHand),  ← 손 인식 성공 시
      PlaceAtStaging                          ← fallback
  )
```

---

## 안전 고려사항

- **속도 제한**: place_on_hand 실행 시 action scale 0.2 이하
- **손 움직임 감지**: `/hand/pose` 변화량 threshold 초과 시 즉시 abort
- **접근 방식**: 손바닥 위 일정 높이에서 그리퍼를 열어 공구를 내려놓음 (직접 접촉 최소화)
- **Force 모니터링**: 접근 중 joint torque threshold 초과 시 정지

---

## 구현 난이도

| 항목 | 난이도 | 비고 |
|------|--------|------|
| `hand_node.py` (MediaPipe + RealSense 융합) | 🟡 보통 | 핵심 작업 |
| 손바닥 방향 판단 | 🟡 보통 | MediaPipe 21 landmarks 활용 |
| `place_on_hand.py` unit action | 🟢 쉬움 | place_at_staging 변형 |
| BT 노드 연결 | 🟢 쉬움 | 기존 BT 구조 확장 |
| 안전 로직 | 🟡 보통 | 속도/토크 제한 |

기존 인프라(hand_eye_loader, RealSense, BT)가 모두 구축되어 있어
`hand_node.py` 작성이 구현의 핵심이다.

---

## 참고 레포

- [Mediapipe-Hand-ROS](https://github.com/DaeyunJang/Mediapipe-Hand-ROS) — MediaPipe + ROS2 TF 브로드캐스트 구현 참고
