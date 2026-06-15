#!/usr/bin/env python3
import cv2
import numpy as np
import yaml

cfg = yaml.safe_load(open('/home/iys/Final_project/config/c270_camera_info.yaml'))['intrinsics']
rt  = yaml.safe_load(open('/home/iys/Final_project/config/runtime.yaml')).get('calibration', {})

K = np.array([[cfg['fx'],0,cfg['cx']],[0,cfg['fy'],cfg['cy']],[0,0,1]], dtype=np.float64)
D = np.array(cfg['coeffs'], dtype=np.float64)

sq_x = int(rt.get('charuco_squares_x', 5))
sq_y = int(rt.get('charuco_squares_y', 7))
sq_m = float(rt.get('charuco_square_size_m', 0.038))
mk_m = float(rt.get('charuco_marker_size_m', 0.0315))
d_id = int(rt.get('aruco_dict_id', cv2.aruco.DICT_4X4_50))

aruco_dict   = cv2.aruco.Dictionary_get(d_id)
aruco_params = cv2.aruco.DetectorParameters_create()
board = cv2.aruco.CharucoBoard_create(sq_x, sq_y, sq_m, mk_m, aruco_dict)

cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

WIN = 'C270 Preview — q:종료'
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 640, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    disp = frame.copy()

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(disp, corners, ids)
        ret2, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board, K, D)
        if ret2 >= 4:
            cv2.aruco.drawDetectedCornersCharuco(disp, cc, ci)
            cv2.putText(disp, f'OK  corners={ret2}', (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            cv2.putText(disp, f'corners={ret2}  (min 4)', (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
    else:
        cv2.putText(disp, 'NO MARKER', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    cv2.imshow(WIN, disp)
    key = cv2.waitKey(30) & 0xFF
    if key in (ord('q'), 27):
        break

cap.release()
cv2.destroyAllWindows()
