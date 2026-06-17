# C270 핸드-아이 캘리브레이션 (eye-in-hand)

## 개요

| 항목 | 내용 |
|------|------|
| 카메라 위치 | 그리퍼 몸통 고정 (eye-in-hand) |
| 사용 마커 | ID 0 (5cm, DICT_4X4_50) |
| 마커 배치 | 작업 테이블 위 고정 (로봇이 마커를 내려다보는 구조) |
| 결과 파일 | `config/c270_hand_eye.yaml` |
| 상태 | 🔄 진행 중 |

---

## 마커 배치 안내

```
      [그리퍼 카메라 C270]
             ↓ 바라봄
  ┌──────────────────────┐
  │                      │
  │    ┌────────┐        │
  │    │  ID 0  │        │  ← 5×5cm, 테이블 위 평평하게 고정
  │    │ (5cm)  │        │     테이프로 움직이지 않게 고정 필수
  │    └────────┘        │
  │                      │
  └──────────────────────┘
         작업 테이블
```

- 마커는 **테이블 위에 평평하게** 고정 (기울어지면 pose 오차 증가)
- 로봇 작업 반경 내 중앙 근처에 배치
- 수집 중 마커 위치 절대 이동 금지

---

## 수집 절차

```
사전 조건
  1. 인트린직 캘리브레이션 완료 (config/c270_camera_info.yaml 존재)
  2. ID 0 마커 테이블에 고정
  3. DART 직접교시 모드 활성화

수집
  python3 scripts/c270_handeye_collect.py

  ① 로봇을 직접교시로 이동 (C270이 마커를 볼 수 있는 자세)
  ② 카메라 창에 초록 OK + 마커 인식 확인
  ③ ENTER → DART 화면 TCP 값 입력 (X Y Z Rx Ry Rz)
     단위: X Y Z = mm,  Rx Ry Rz = deg (ZYZ 오일러)
  ④ 15~25쌍 반복 (다양한 방향·거리 권장)

계산
  python3 scripts/compute_handeye_opencv.py
  또는
  python3 scripts/compute_handeye_opencv.py --all   # 5가지 알고리즘 비교
```

---

## 자세 다양성 권장 사항

| 항목 | 권장값 |
|------|--------|
| 자세 수 | 15~25쌍 |
| 마커 거리 | 0.30 ~ 0.85m |
| 마커 기울기 | 카메라 정면 기준 ±22° 이내 |
| 로봇 자세 | J4/J5/J6 방향 다양하게 변화 |

---

## 검증 방법

수집 완료 후 `compute_handeye_opencv.py` 출력에서 확인:

| 지표 | 목표 | 비고 |
|------|------|------|
| AXB 잔차 mean | < 2.0mm | 낮을수록 좋음 |
| AXB 잔차 max | < 5.0mm | 이상값 자세 제거 후 재시도 |
| 샘플 수 (필터 후) | 12개 이상 | 필터 탈락 많으면 자세 재수집 |

---

## 결과 기록

| 날짜 | 샘플 수 | AXB mean | AXB max | 알고리즘 | 비고 |
|------|---------|----------|---------|---------|------|
| (미완료) | — | — | — | — | — |

---

## 관련 파일

```
scripts/
├── c270_handeye_collect.py     ← 데이터 수집 (DART TCP 수동 입력)
├── c270_handeye_capture.py     ← 이미지만 캡처 (tw 파일 연동 방식)
├── compute_handeye_opencv.py   ← 변환행렬 계산 → config/c270_hand_eye.yaml
├── calib_poses.py              ← handeye_calib_motion.py 전용 자세 목록 (자동 이동 방식)
├── handeye_calib_motion.py     ← ROS2로 자동 자세 이동 (calib_poses.py 사용)
├── samples_c270_handeye/       ← 수집 이미지 (gitignore, 로컬 전용)
└── multi_item/markers/
    ├── marker_0_ID0_ceiling.png        ← ID 0 인쇄 파일
    └── aruco_markers.pdf               ← 전체 묶음
```
