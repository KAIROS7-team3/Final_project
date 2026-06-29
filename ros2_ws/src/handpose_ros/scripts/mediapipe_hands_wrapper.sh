#!/bin/bash
# mediapipe_hands_node를 전용 venv에서 실행하는 래퍼.
# 검증 조합: numpy 1.26.4 + mediapipe 0.10.14 + opencv-contrib 4.11.0.86 + protobuf 4.x.
# mediapipe·cv_bridge·cv2 모두 numpy 1.x C-ABI로 빌드돼 있으므로 venv numpy도
# 반드시 1.x로 고정해야 한다. numpy 2.x로 올리면 cv_bridge import 시
# `_ARRAY_API not found`로 죽고 mediapipe 본체도 크래시한다. (README 핸드오버 설치 참고)

VENV_PYTHON="/home/user/Final_project/handpose_venv/bin/python"
WS="/home/user/Final_project/ros2_ws"

export PYTHONPATH="\
${WS}/install/handpose_interfaces/local/lib/python3.10/dist-packages:\
${WS}/install/handpose_ros/lib/python3.10/site-packages:\
/opt/ros/humble/lib/python3.10/site-packages:\
/opt/ros/humble/local/lib/python3.10/dist-packages"

export LD_LIBRARY_PATH="\
${WS}/install/handpose_interfaces/lib:\
/opt/ros/humble/lib:\
/opt/ros/humble/opt/rviz_ogre_vendor/lib:\
${LD_LIBRARY_PATH}"

exec "$VENV_PYTHON" -m handpose_ros.mediapipe_hands_node "$@"
