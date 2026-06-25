#!/bin/bash
# colcon build 후 voice entry point shebang을 fp Python으로 교체
FP_PYTHON=/home/user/miniconda3/envs/fp/bin/python
for f in /home/user/Final_project/ros2_ws/install/voice/lib/voice/*; do
  sed -i "1s|#!/usr/bin/python3|#!$FP_PYTHON|" "$f"
done
echo "shebang 패치 완료 → $FP_PYTHON"
