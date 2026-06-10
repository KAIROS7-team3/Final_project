# ArUco 마커

딕셔너리: `DICT_4X4_50` | 한 변 길이: **5cm** (실측, ±1mm 이내)

## 파일 목록

| 파일 | 마커 ID | 배치 위치 | 용도 |
|------|---------|----------|------|
| `marker_0_ID0_ceiling.png` | 0 | 천장 고정 | 탑뷰 D455f 캘리브 / C270 eye-in-hand 캘리브 |
| `marker_0_ID0_ceiling_clean.png` | 0 | 동일 | 여백 최소화 버전 (인쇄 권장) |
| `marker_1_ID1_drawer_layer0.png` | 1 | 공구함 서랍 레이어 0 | 탑뷰 공구함 위치 인식 |
| `marker_1_ID1_drawer_layer0_clean.png` | 1 | 동일 | 여백 최소화 버전 (인쇄 권장) |
| `marker_2_ID2_drawer_layer1.png` | 2 | 공구함 서랍 레이어 1 | 탑뷰 공구함 위치 인식 |
| `marker_2_ID2_drawer_layer1_clean.png` | 2 | 동일 | 여백 최소화 버전 (인쇄 권장) |
| `aruco_markers.pdf` | 0·1·2 | — | 전체 인쇄용 묶음 |

## 인쇄 주의사항

- **`_clean.png` 버전으로 인쇄** — 흰 테두리 여백이 적어 실측값 오차가 줄어듦
- 인쇄 후 마커 한 변 실측 필수 — `config/vision.yaml` `marker_size_m` 및 `config/runtime.yaml` `aruco_marker_size_m` 과 일치 확인
- 라미네이팅 권장 (반사광 주의 — 무광 라미네이팅 사용)

## 참조하는 코드

| 파일 | 참조 방식 |
|------|----------|
| `config/vision.yaml` | `marker_size_m`, `dictionary` 설정값 |
| `config/runtime.yaml` | `aruco_marker_size_m`, `aruco_dict_id` 설정값 |
| `scripts/c270_handeye_collect.py` | `ARUCO_TARGET_ID = 0`, `ARUCO_DICT_ID` |
| `scripts/c270_handeye_capture.py` | `DICT_4X4_50` 하드코딩 |
| `scripts/examples/example_aruco_detection.py` | `DICT_4X4_50` |
| `ros2_ws/src/vision/vision/marker_scan_node.py` | `config/vision.yaml` 경유 |
