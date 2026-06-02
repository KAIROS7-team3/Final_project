# 탑뷰 ArUco 마커 스캔 결과 — MarkerScanNode 발행
#
# marker_ids[i] 에 대응하는 포즈: poses_robot[i]
#   position: base_link 기준 [m]  /  orientation: quaternion [x,y,z,w]
#   calibrated=true  → base_link 기준
#   calibrated=false → camera_color_optical_frame 기준 (hand-eye 미완료)
#
# place_zone_radius: 마커 중심 기준 공구 place 허용 반경 [m]
#   config/vision.yaml aruco.place_zone_radius_mm × 0.001 변환값

std_msgs/Header        header
int32[]                marker_ids
geometry_msgs/Pose[]   poses_robot
float32                place_zone_radius
bool                   calibrated
