#!/usr/bin/env bash
# Hand-Eye 캘리브레이션 절차 스크립트 (eye-to-hand, D455f + Doosan e0509)
#
# 합격 기준 (G1 → 2 게이트):
#   - reprojection error < 1.0 px
#   - position error     < 5 mm
#   - orientation error  < 2°
#
# 사전 조건:
#   - ROS2 Humble 소스 완료 (source ~/Final_project/ros2_ws/install/setup.bash)
#   - doosan-robot2 드라이버 실행 중 (담당: B)
#   - D455f USB 3.x 연결 확인 (ls /dev/realsense)
set -euo pipefail

ROS_WS="${HOME}/Final_project/ros2_ws"
HANDEYE_RESULT="${HOME}/.ros/easy_handeye2/d455f_e0509.yaml"
PROJECT_CONFIG="${HOME}/Final_project/config/hand_eye.yaml"

echo "===== §0. CharUco 보드 생성 (최초 1회) ====="
echo "아래 명령으로 charuco_board.png를 A4로 인쇄하세요."
cat <<'PYTHON'
python3 -c "
import cv2
board = cv2.aruco.CharucoBoard(
    (8, 6), 0.04, 0.03,
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
)
img = board.generateImage((1600, 1200))
cv2.imwrite('charuco_board.png', img)
print('charuco_board.png 저장 완료 — A4 실물 크기로 인쇄')
"
PYTHON

echo ""
echo "===== §1. easy_handeye2 빌드 (최초 1회) ====="
cat <<'CMD'
cd ~/Final_project/ros2_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easy_handeye2
source install/setup.bash
CMD

echo ""
echo "===== §2. 캘리브레이션 런치 ====="
echo "터미널 A: doosan-robot2 드라이버 (담당: B가 실행)"
echo "터미널 B: 아래 명령 실행"
cat <<CMD
source ${ROS_WS}/install/setup.bash
ros2 launch vision handeye_calibration.launch.py
CMD

echo ""
echo "===== §3. 샘플 수집 (최소 15개, 권장 25개+) ====="
cat <<'GUIDE'
1. GUI에서 보드 검출 확인 (초록 박스 표시 필수)
2. 로봇을 아래 순서로 다양한 자세로 이동:
     - J1: ±45° 범위에서 5 단계
     - J2/J3: 상하 경사 변화
     - J5: 보드 기울기 변화
   → 보드가 카메라에 완전히 보일 것 (부분 가림 샘플 제외)
3. 각 자세에서 GUI "Take sample" 클릭
4. 샘플 수가 25 이상이 되면 "Compute" 클릭
GUIDE

echo ""
echo "===== §4. 결과 저장 및 config/hand_eye.yaml 갱신 ====="
cat <<CMD
# easy_handeye2 자동 저장 또는:
ros2 service call /easy_handeye2/save std_srvs/srv/Empty
CMD

echo ""
echo "easy_handeye2 결과 파일 위치: ${HANDEYE_RESULT}"
echo "결과를 ${PROJECT_CONFIG} 에 아래 형식으로 옮겨 기입하세요."
cat <<'YAML'
transformation:
  rotation:
    x: <결과값>
    y: <결과값>
    z: <결과값>
    w: <결과값>
  translation:
    x: <결과값>  # [m]
    y: <결과값>  # [m]
    z: <결과값>  # [m]

metadata:
  calibration_date: "<YYYY-MM-DD>"
  sample_count: <n>
  reprojection_error_px: <값>
  position_error_mm: <값>
  orientation_error_deg: <값>
  operator: "<이름>"
YAML

echo ""
echo "===== §5. 검증 ====="
cat <<'GUIDE'
알려진 슬롯 위치에 ArUco 마커를 두고:
  python3 -c "
  from pathlib import Path
  import numpy as np
  from vision.hand_eye_loader import load_transform, camera_to_base

  T = load_transform(Path('config/hand_eye.yaml'))
  # 카메라로 마커 검출 후 depth → 3D 점 입력
  point_cam = np.array([X, Y, Z])   # 카메라 좌표 [m]
  point_base = camera_to_base(point_cam, T)
  print(f'base_link 좌표: {point_base}')
  # 실측값과 비교 → 오차 < 5mm 확인
  "
GUIDE

echo ""
echo "캘리브레이션 완료 후 robot-arm-project.md Phase 1 항목 체크 필요."
