---
name: realsense-d455f
description: >
  Intel RealSense D455f RGB-D 카메라 사용 가이드 — 사양·pyrealsense2 API·
  realsense-ros ROS2 wrapper·frame 정렬(depth↔RGB)·캘리브레이션·흔한 깊이 품질 문제.
  D455f 스트림 처리, depth 데이터 활용, hand-eye 캘리브레이션 준비 시 활성화.
when_to_use: >
  D455f 노드 작성, pyrealsense2 직접 호출(Track C), depth-RGB 정렬,
  depth 노이즈/구멍 문제 디버깅 시.
  (카메라-로봇 핸드-아이 캘리브레이션 절차는 hand-eye-calibration 스킬 전담)
---

# Intel RealSense D455f 가이드

> 이 프로젝트에서 D455f는 공구 인식(YOLOv8 + 6D Pose, Track A/B) 및 VLA 입력(Raw RGB-D, Track C)으로 사용. 룰: [`.claude/rules/engineering.md`](../rules/engineering.md) E-1 (좌표/단위), [`.claude/rules/safety.md`](../rules/safety.md).

## 1. 하드웨어 사양

| 항목 | D455f 값 |
|------|----------|
| RGB 해상도 | 1280×720 @ 30fps (최대 1920×1080 @ 30fps) |
| Depth 해상도 | 1280×720 @ 30fps |
| Depth 측정 범위 | 0.6 m ~ 6 m (recommended 0.4~3m) |
| Depth 정확도 | ≤2% @ 4m |
| FOV | RGB 90°×65°, Depth 87°×58° |
| 베이스라인 | 95mm (D435 대비 길어서 원거리 정확도 ↑) |
| IMU | 내장 (gyro + accel) |
| 글로벌 셔터 | Depth 카메라만 (RGB는 rolling shutter) |
| **f 모델 특징** | IR 패시브 + 광시야 (filter 도장형) |
| 인터페이스 | USB 3.1 Gen 1 (5 Gbps) |

> 이 프로젝트는 0.5~1.5m 범위의 공구함 작업 — D455f sweet spot. D435보다 정확도 우수.

## 2. 설치

### Linux (Ubuntu 22.04)
```bash
# librealsense
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# Python binding
pip install pyrealsense2

# ROS2 wrapper
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

### udev (안정적인 디바이스 노드)
```bash
# /etc/udev/rules.d/99-realsense-d455.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE="0666", SYMLINK+="realsense"
```
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 3. Python SDK 사용 (Track C 전용)

### 기본 스트림
```python
import pyrealsense2 as rs
import numpy as np

# 파이프라인 + 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 시작
profile = pipeline.start(config)

# Depth 스케일 (depth raw → meters)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()   # 보통 0.001 (1mm)

try:
    frames = pipeline.wait_for_frames(timeout_ms=1000)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        raise RuntimeError("frame 수신 실패")

    rgb = np.asanyarray(color_frame.get_data())          # (H, W, 3) uint8 BGR
    depth_raw = np.asanyarray(depth_frame.get_data())    # (H, W) uint16
    depth_m = depth_raw * depth_scale                    # (H, W) float64 meters

finally:
    pipeline.stop()
```

### Frame 정렬 (Depth → RGB) — 필수

Depth와 RGB 센서는 물리적으로 떨어져 있어 정렬 필요.

```python
# 정렬 객체 생성 — RGB 좌표계로 depth 정렬
align = rs.align(rs.stream.color)

frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)

aligned_color = aligned_frames.get_color_frame()
aligned_depth = aligned_frames.get_depth_frame()

# 이제 같은 픽셀 (u, v) 좌표가 같은 3D 점에 대응
rgb = np.asanyarray(aligned_color.get_data())
depth = np.asanyarray(aligned_depth.get_data())
```

### 2D 픽셀 → 3D 좌표 변환
```python
# Intrinsics
depth_intrinsics = aligned_depth.profile.as_video_stream_profile().intrinsics

# 픽셀 (u, v) + depth → 3D 점 (카메라 좌표계, meters)
u, v = 640, 360   # bbox 중심 등
depth_at_pixel = aligned_depth.get_distance(u, v)  # 자동 깊이 스케일 적용

point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [u, v], depth_at_pixel)
# point_3d = [x, y, z] in meters, camera frame

# 3D → 픽셀 (역변환)
pixel = rs.rs2_project_point_to_pixel(depth_intrinsics, point_3d)
```

## 4. ROS2 wrapper 사용 (Track A/B)

### 런칭
```bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=d455f \
    camera_name:=d455f \
    enable_color:=true \
    enable_depth:=true \
    align_depth.enable:=true \
    rgb_camera.color_profile:=1280x720x30 \
    depth_module.depth_profile:=1280x720x30
```

### 주요 토픽
| 토픽 | 타입 | 비고 |
|------|------|------|
| `/d455f/color/image_raw` | sensor_msgs/Image | RGB |
| `/d455f/depth/image_rect_raw` | sensor_msgs/Image | Depth (정렬 안 됨) |
| `/d455f/aligned_depth_to_color/image_raw` | sensor_msgs/Image | **Depth 정렬됨 — 권장** |
| `/d455f/color/camera_info` | sensor_msgs/CameraInfo | RGB intrinsics |
| `/d455f/aligned_depth_to_color/camera_info` | sensor_msgs/CameraInfo | 정렬된 depth intrinsics |
| `/d455f/extrinsics/depth_to_color` | realsense2_camera_msgs/Extrinsics | RGB↔Depth 변환 |
| `/d455f/imu` | sensor_msgs/Imu | (활성화 시) |

### 노드 구독
```python
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

bridge = CvBridge()

def rgb_callback(msg: Image):
    rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # ...

def depth_callback(msg: Image):
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    depth_m = depth.astype(np.float32) * 0.001   # uint16 → meters
```

### message_filters로 RGB+Depth 동기화
```python
from message_filters import Subscriber, ApproximateTimeSynchronizer

rgb_sub = Subscriber(self, Image, '/d455f/color/image_raw')
depth_sub = Subscriber(self, Image, '/d455f/aligned_depth_to_color/image_raw')

sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.05)
sync.registerCallback(self.on_rgbd)

def on_rgbd(self, rgb_msg, depth_msg):
    rgb = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
    # ...
```

## 5. Hand-Eye 캘리브레이션 (이 프로젝트 필수)

D455f가 로봇 엔드이펙터 또는 외부 고정 위치에 장착됨 → 카메라 frame과 로봇 frame 변환 행렬 필요.

### 두 가지 마운팅 방식
| 방식 | 변환 | 캘리브 결과 저장 |
|------|------|------------------|
| **Eye-in-Hand** (로봇 손목에 장착) | `camera_link → tool_link` | `config/hand_eye.yaml` |
| **Eye-to-Hand** (외부 고정) | `camera_link → robot_base_link` | `config/hand_eye.yaml` |

### 캘리브레이션 절차
1. **체커보드 / AprilTag 마커 준비** (e.g., 6x9 squares × 25mm)
2. **로봇을 다양한 포즈로 이동** + 각 포즈에서 마커 이미지 + 로봇 joint 기록 (15~30 샘플)
3. **OpenCV `calibrateHandEye()` 또는 `easy_handeye2` ROS2 패키지** 실행
4. **결과 저장**:
```yaml
# config/hand_eye.yaml
schema_version: 1
mounting: eye_in_hand          # eye_in_hand | eye_to_hand
camera_to_tool:
  translation_m: [0.05, 0.02, -0.10]
  quaternion: [0.0, 0.0, 0.0, 1.0]
```

### easy_handeye2 사용
```bash
sudo apt install ros-humble-easy-handeye2
ros2 launch easy_handeye2 calibrate.launch.py \
    calibration_type:=eye_in_hand \
    name:=d455f_e0509
# GUI 표시 → "Take Sample" 반복 → "Compute" → 결과 저장
```

## 6. Depth 품질 개선

### 문제 — 흔한 증상
- **반사 표면 (금속 공구)** → 깊이 구멍 발생
- **검은 / 투명 표면** → 깊이 측정 실패
- **너무 가까움 (<0.4m)** → 깊이 부정확
- **흔들림 / 직사광** → 노이즈 증가

### 후처리 필터 (pyrealsense2)
```python
# Decimation — 다운샘플 (성능 ↑)
decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 2)

# Threshold — 범위 클리핑
threshold = rs.threshold_filter()
threshold.set_option(rs.option.min_distance, 0.4)
threshold.set_option(rs.option.max_distance, 2.0)

# Spatial — 공간 평활
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 2)

# Temporal — 시간 평활 (이전 프레임과 평균)
temporal = rs.temporal_filter()

# Hole filling — 작은 구멍 메움
hole_filling = rs.hole_filling_filter()
hole_filling.set_option(rs.option.holes_fill, 1)   # 0=무, 1=2pixel, 2=4pixel...

# 적용 (순서 중요)
depth = decimation.process(depth)
depth = threshold.process(depth)
depth = spatial.process(depth)
depth = temporal.process(depth)
depth = hole_filling.process(depth)
```

### High Density vs High Accuracy 프리셋
```python
# preset = 0 (Custom), 1 (Default), 2 (Hand), 3 (HighAccuracy), 4 (HighDensity), 5 (MediumDensity)
sensor = profile.get_device().first_depth_sensor()
sensor.set_option(rs.option.visual_preset, 3)   # HighAccuracy 권장 (이 프로젝트)
```

## 7. 좌표계

```
camera_link (D455f 광학 중심)
  · 표준 카메라 좌표: x→오른쪽, y→아래, z→앞 (depth 방향)
  ↓ hand_eye.yaml의 변환
tool_link (eye-in-hand) 또는 robot_base_link (eye-to-hand)
```

ROS2 표준 frame과 다름 주의 — RealSense는 광학 좌표계 사용.

```python
# REP-105 ROS frame 사용 시 변환 필요
# RealSense optical (x→right, y→down, z→forward)
# →  ROS standard (x→forward, y→left, z→up)

# realsense-ros wrapper는 자동으로 변환 + tf 발행
# pyrealsense2 직접 사용 시 수동 변환 필요
```

## 8. ROS2 TF 통합

```bash
# 실행 시 자동 TF 발행
ros2 run tf2_ros static_transform_publisher \
    --x 0.05 --y 0.02 --z -0.10 \
    --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id tool_link --child-frame-id d455f_link

# 또는 robot URDF/xacro에 포함
```

```python
# 코드에서 변환 조회
from tf2_ros import Buffer, TransformListener

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, self)

# camera 좌표의 점 → robot_base_link로 변환
trans = tf_buffer.lookup_transform("robot_base_link", "d455f_link", rclpy.time.Time())
# trans 사용해 점 변환
```

## 9. 디버깅 도구

```bash
# 카메라 인식 확인
rs-enumerate-devices

# 실시간 뷰어
realsense-viewer

# ROS2에서 RGB 시각화
rqt_image_view /d455f/color/image_raw

# Depth as point cloud
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
# RViz: PointCloud2 → /d455f/depth/color/points
```

## 10. 흔한 함정

### ❌ 정렬 안 된 depth 사용
- depth(u,v)와 RGB(u,v)가 다른 3D 점을 가리킴
- ✅ 항상 `aligned_depth_to_color` 사용

### ❌ Depth scale 무시
```python
depth_raw = np.asanyarray(depth_frame.get_data())   # uint16
distance = depth_raw[v, u]   # mm? meters? 모호함
```
✅ 항상 `depth_scale` 곱하기 또는 `depth_frame.get_distance(u, v)` 사용

### ❌ 너무 가까이서 사용
- 0.4m 미만은 측정 불가 영역 — 0.5~1.5m 권장 (이 프로젝트의 작업거리에 적합)

### ❌ 직사광 / 강한 IR 광원
- D455f는 IR 패턴 사용 — 강한 IR 환경에서 노이즈 증가
- ✅ 실내 일반 조명 권장

### ❌ USB 2.0 포트
- D455f는 USB 3.1 필수 — 2.0 포트에서는 저해상도/저프레임만 가능
- ✅ `lsusb -t`로 SuperSpeed 확인

### ❌ 캘리브레이션 없이 사용
- 카메라-로봇 frame 변환 없으면 모든 좌표 계산 부정확
- ✅ 시스템 초기 셋업 시 hand-eye 캘리브 필수

## 11. 참고

- librealsense 문서: <https://github.com/IntelRealSense/librealsense>
- pyrealsense2 API: <https://intelrealsense.github.io/librealsense/python_docs/>
- realsense-ros: <https://github.com/IntelRealSense/realsense-ros>
- D455 데이터시트: <https://www.intelrealsense.com/depth-camera-d455/>
- easy_handeye2: <https://github.com/marcoesposito1988/easy_handeye2>
- 프로젝트 룰: [`.claude/rules/engineering.md`](../rules/engineering.md), [`.claude/rules/safety.md`](../rules/safety.md)
