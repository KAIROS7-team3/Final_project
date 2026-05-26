# 좌표계 및 TF Tree

> REP-103 (단위·좌표 규약) 준수. 단위: 위치 **m**, 각도 **rad**, 시간 **s**.
> 캘리브레이션 파일 위치·절차 → [`.claude/skills/hand-eye-calibration/SKILL.md`](../.claude/skills/hand-eye-calibration/SKILL.md)

---

## 1. TF Tree

```
world
└── base_link                    (Doosan e0509 베이스 — 고정)
    ├── link1
    │   └── link2
    │       └── link3
    │           └── link4
    │               └── link5
    │                   └── link6
    │                       └── tool0          (엔드이펙터 플랜지)
    │                           └── gripper_link   (RH-P12-RN 장착점)
    │
    └── camera_link              (RealSense D455f 마운트 — eye-to-hand, 고정)
        └── camera_optical_frame (광학 좌표계: Z 앞, X 오른쪽, Y 아래)
```

**eye-to-hand 구성:** 카메라는 로봇 셀에 고정 마운트. 엔드이펙터에 장착하지 않는다.  
→ `world → camera_link` 변환은 `config/hand_eye.yaml`에서 StaticTransformBroadcaster로 발행.

---

## 2. 주요 Frame 정의

| Frame ID | 부모 | 정의 방법 | 의미 |
|----------|------|----------|------|
| `world` | — | 고정 원점 | 작업 공간 전역 좌표계 |
| `base_link` | `world` | URDF 고정 | Doosan e0509 베이스 플레이트 중심 |
| `tool0` | `link6` | URDF | 엔드이펙터 플랜지 (ISO 9283 기준) |
| `gripper_link` | `tool0` | URDF | RH-P12-RN 그리퍼 장착 기준점 |
| `camera_link` | `world` | `hand_eye.yaml` → StaticTF | D455f 카메라 바디 중심 |
| `camera_optical_frame` | `camera_link` | realsense-ros 드라이버 자동 | Z 앞, X 오른쪽 광학 좌표 |

---

## 3. 좌표 규약 (REP-103)

| 항목 | 규약 |
|------|------|
| 위치 단위 | **m** (코드·config 모두) |
| 각도 단위 | **rad** (joint angle, 회전 모두) |
| X축 | 앞 (Forward) |
| Y축 | 왼쪽 (Left) |
| Z축 | 위 (Up) |
| 회전 표현 | quaternion `[x, y, z, w]` (ROS 표준) |
| 광학 frame | Z 앞, X 오른쪽, Y 아래 (REP-103 optical frame) |

> `config/hand_eye.yaml`의 rotation은 quaternion `[x, y, z, w]` 형식으로 저장.

---

## 4. Hand-Eye 변환

### 구성

- **타입:** eye-to-hand (카메라 고정, 엔드이펙터 추적 불필요)
- **변환:** `world → camera_link`
- **파일:** `config/hand_eye.yaml`

```yaml
# config/hand_eye.yaml 예시 구조
hand_eye_calibration:
  child_frame: camera_link
  parent_frame: world
  rotation:    [x, y, z, w]   # quaternion
  translation: [x, y, z]      # m 단위
  calibrated_at: "YYYY-MM-DD"
  reprojection_error_px: 0.0
```

### 수락 기준

| 지표 | 기준 |
|------|------|
| 재투영 오차 | < 1.0px |
| 위치 오차 | < 5mm |
| 방향 오차 | < 2° |

재캘리브레이션 조건: 카메라 마운트 물리적 변경, 재투영 오차 > 1.5px, 포즈 추정 오차 반복 초과.

---

## 5. 검증 명령

```bash
# TF tree 전체 확인
ros2 run tf2_tools view_frames

# 특정 frame 간 변환 조회
ros2 run tf2_ros tf2_echo base_link camera_optical_frame

# camera_link StaticTF 발행 확인
ros2 topic echo /tf_static --once

# tool0 실시간 위치 확인
ros2 run tf2_ros tf2_echo base_link tool0
```

---

## 6. 주의 사항

- **Track C는 TF 미사용.** `pyrealsense2` 직접 접근 + VLA 모델이 raw RGB-D 처리. 좌표 변환 없음.
- `config/staging_area.yaml`의 Staging Area 좌표는 **`base_link` 기준** (robot_base_link frame).
- `config/toolbox.yaml`의 슬롯 좌표도 **`base_link` 기준**.
- 카메라 내부 파라미터(intrinsics)는 realsense-ros 드라이버가 `/camera/camera_info`로 자동 발행.
