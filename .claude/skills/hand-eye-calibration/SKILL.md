---
name: hand-eye-calibration
description: >
  D455f + Doosan e0509 핸드-아이 캘리브레이션 절차 — eye-to-hand 설정,
  easy_handeye2 사용법, config/hand_eye.yaml 형식, 검증 방법, 재캘리브레이션 기준.
  카메라-로봇 좌표 변환 행렬 획득 및 갱신 시 활성화.
when_to_use: >
  카메라 위치 변경, 정확도 저하, 신규 설치, hand_eye.yaml 갱신,
  6D pose 오차 증가, easy_handeye2 실행, 캘리브레이션 검증 시.
---

# 핸드-아이 캘리브레이션 (D455f + Doosan e0509)

> 이 프로젝트는 **eye-to-hand** 구성: D455f가 로봇 베이스 근처 고정 위치에 장착.
> 캘리브레이션 결과는 `config/hand_eye.yaml`에 저장. 룰: [`.claude/rules/engineering.md`](../rules/engineering.md) E-1 (좌표 frame 명시).

## 1. 캘리브레이션 유형

| 유형 | 카메라 위치 | 이 프로젝트 |
|------|-----------|-----------|
| **eye-to-hand** | 고정 (작업공간 외부) | ✅ 사용 |
| eye-in-hand | 엔드이펙터에 부착 | ❌ |

eye-to-hand에서 구하는 변환:
```
T_camera_to_base: 카메라 좌표계 → 로봇 베이스 좌표계
```

## 2. 필요한 것

- **캘리브레이션 보드**: ArUco 마커 보드 (charuco 권장) 또는 체커보드
  ```bash
  # CharUco 보드 생성 (A4 인쇄)
  python -c "
  import cv2
  board = cv2.aruco.CharucoBoard((8, 6), 0.04, 0.03,
                                  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
  img = board.generateImage((800, 600))
  cv2.imwrite('charuco_board.png', img)
  "
  ```
- **ROS2 패키지**: `easy_handeye2`, `realsense-ros`, `doosan-robot2`

## 3. easy_handeye2 설치

```bash
cd ~/ros2_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easy_handeye2
source install/setup.bash
```

## 4. 캘리브레이션 절차

### Step 1: 런치 파일 준비
```yaml
# launch/handeye_calibration.launch.py (개요)
# 실행 노드:
# 1. realsense2_camera (D455f)
# 2. doosan_robot2 (e0509 드라이버)
# 3. easy_handeye2 캘리브레이션 노드
```

```bash
# 캘리브레이션 런치
ros2 launch easy_handeye2 calibrate.launch.py \
  eye_on_hand:=false \
  robot_base_frame:=base_link \
  robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=aruco_marker_frame
```

### Step 2: 포즈 수집 (최소 15개 권장)
```
1. 로봇을 다양한 자세로 이동 (관절 각도 크게 변화)
   - 최소 15개 자세, 권장 25개+
   - 보드가 카메라 시야 내에 완전히 포함되어야 함
   - 관절 1, 2, 4, 5를 주로 변경 (다양한 각도 확보)

2. 각 자세에서:
   a. 마커 검출 확인 (초록 박스 표시)
   b. easy_handeye2 GUI에서 "Take sample" 클릭
   c. 샘플 수 증가 확인

3. 25개 이상 수집 후 "Compute" 클릭
```

### Step 3: 결과 저장
```bash
# easy_handeye2가 자동 저장 또는 수동 저장
ros2 service call /easy_handeye2/save std_srvs/srv/Empty
# → ~/.ros/easy_handeye2/<calibration_name>.yaml
```

## 5. config/hand_eye.yaml 형식

```yaml
# config/hand_eye.yaml
# frame: camera_color_optical_frame → base_link 변환
# 캘리브레이션 일자: 2026-05-23
# 방법: eye-to-hand, easy_handeye2, charuco 보드 25 샘플

schema_version: 1

transformation:
  # 쿼터니언 [x, y, z, w]
  rotation:
    x: -0.023
    y:  0.012
    z:  0.701
    w:  0.712
  # 위치 [m], base_link 기준
  translation:
    x: 0.412
    y: -0.185
    z: 1.023

# 재캘리브레이션 기준
metadata:
  calibration_date: "2026-05-23"
  sample_count: 25
  reprojection_error_px: 0.42   # < 1.0 목표
  operator: "kg"
```

## 6. 변환 행렬 사용 (Python)

```python
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from pathlib import Path

def load_hand_eye(path: Path = Path("config/hand_eye.yaml")) -> np.ndarray:
    """4×4 변환 행렬 반환: camera_frame → base_link."""
    with path.open() as f:
        cfg = yaml.safe_load(f)["transformation"]

    rot = cfg["rotation"]
    trans = cfg["translation"]

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([rot["x"], rot["y"], rot["z"], rot["w"]]).as_matrix()
    T[:3, 3] = [trans["x"], trans["y"], trans["z"]]
    return T

T_cam_to_base = load_hand_eye()

def camera_to_base(point_camera: np.ndarray) -> np.ndarray:
    """카메라 좌표 → 로봇 베이스 좌표 변환.
    point_camera: (3,) [m]
    """
    p_hom = np.append(point_camera, 1.0)
    return (T_cam_to_base @ p_hom)[:3]

# 사용: YOLOv11s + depth로 구한 3D 좌표를 로봇 좌표로 변환
tool_pos_cam = np.array([0.12, -0.05, 0.65])   # 카메라 좌표 [m]
tool_pos_base = camera_to_base(tool_pos_cam)    # 베이스 좌표 [m]
```

## 7. ROS2에서 TF로 사용

```python
from geometry_msgs.msg import TransformStamped
import tf2_ros

class HandEyePublisher(Node):
    def __init__(self):
        super().__init__("hand_eye_publisher")
        self._bc = tf2_ros.StaticTransformBroadcaster(self)
        self._publish()

    def _publish(self):
        T = load_hand_eye_cfg()
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.child_frame_id = "camera_color_optical_frame"
        msg.transform.translation.x = T.translation.x
        msg.transform.rotation.x = T.rotation.x
        # ... (나머지 필드)
        self._bc.sendTransform(msg)
```

## 8. 검증

### 재현성 테스트
```python
# 알려진 위치에 마커를 두고 측정
# 공구함 슬롯 위치를 독립적으로 측정한 값과 비교

known_pos_base = np.array([0.55, 0.10, 0.08])   # 실측값 [m]
detected_pos_base = camera_to_base(detect_marker())

error_m = np.linalg.norm(known_pos_base - detected_pos_base)
print(f"캘리브레이션 오차: {error_m*1000:.1f}mm")
# 목표: < 5mm (파지 성공률에 직결)
```

### 합격 기준
| 지표 | 목표 |
|------|------|
| 재투영 오차 | < 1.0 pixel |
| 위치 오차 | < 5mm |
| 자세 오차 | < 2° |

## 9. 재캘리브레이션 기준

아래 상황에서 즉시 재캘리브레이션:
- 카메라 마운트 위치/각도 변경
- 파지 성공률이 갑자기 5% 이상 하락
- 로봇 베이스 이동 후
- 6개월 경과 (열팽창, 진동 누적)
- `reprojection_error > 1.5px` 경고 발생

## 10. 흔한 함정

### ❌ 샘플 수 부족 (< 15개)
- 수학적으로 해는 나오지만 정확도 낮음
- ✅ 25개 이상, 다양한 자세 (특히 관절 각도 크게 변화)

### ❌ 동일한 평면에서만 샘플 수집
- 보드를 항상 수평으로만 두면 회전 캘리브레이션 정확도 저하
- ✅ 보드를 기울이고 다양한 거리에서 수집

### ❌ 캘리브레이션 결과를 코드에 하드코딩
```python
T = np.array([[0.7, -0.1, ...]])   # ❌
```
✅ 항상 `config/hand_eye.yaml`에서 로드

### ❌ frame_id 혼용
- `camera_color_optical_frame` vs `camera_link` 혼용
- ✅ yaml 주석에 from/to frame 명시, 코드에서 동일 frame_id 사용

### ❌ 재캘리브레이션 없이 카메라 재장착
- 나사 1개만 다시 조여도 수 mm 오차 발생 가능
- ✅ 장착 후 항상 검증 (§8)

## 11. 참고

- easy_handeye2: <https://github.com/marcoesposito1988/easy_handeye2>
- Hand-Eye Calibration 이론: Tsai-Lenz, Park-Martin 방법
- 관련 스킬: [`realsense-d455f`](realsense-d455f.md), [`doosan-e0509`](doosan-e0509.md)
- 설정 파일: `config/hand_eye.yaml`
