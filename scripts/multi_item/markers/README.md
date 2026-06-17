# ArUco 마커

딕셔너리: `DICT_4X4_50` | 한 변 길이: **5cm** (실측, ±1mm 이내)

## 파일 목록

| 파일 | 마커 ID | 배치 위치 | 용도 |
|------|---------|----------|------|
| `marker_0_ID0_ceiling.png` | 0 | 공구함 전면 부착 | C270 그리퍼캠 전면 기준점 |
| `marker_0_ID0_ceiling_clean.png` | 0 | 동일 | 여백 최소화 버전 (인쇄 권장) |
| `aruco_markers.pdf` | 0 | — | 인쇄용 |

> **ID 2 마커** — 천장 고정 (실물 부착 완료). 별도 파일 없음.
> `config/toolbox.yaml` `aruco.marker_id: 2` 로 관리.

## 마커 배치 현황

| ID | 위치 | 크기 | config |
|----|------|------|--------|
| **0** | 공구함 전면 / 서랍 아랫층 (layer 0) | 전면 4cm / 서랍 5cm | `toolbox.yaml` `aruco_front` |
| **1** | 서랍 윗층 (layer 1) | 5cm | — |
| **2** | 천장 고정 | 5cm | `toolbox.yaml` `aruco` |

## 인쇄 주의사항

- **`_clean.png` 버전으로 인쇄** — 흰 테두리 여백이 적어 실측값 오차가 줄어듦
- 인쇄 후 마커 한 변 실측 필수 — `config/vision.yaml` `marker_size_m` 및 `config/runtime.yaml` `aruco_marker_size_m` 과 일치 확인
- 라미네이팅 권장 (반사광 주의 — 무광 라미네이팅 사용)

## 참조하는 코드

| 파일 | 참조 방식 |
|------|----------|
| `config/vision.yaml` | `marker_size_m`, `dictionary` 설정값 |
| `config/runtime.yaml` | `aruco_marker_size_m`, `aruco_dict_id` |
| `scripts/c270_handeye_collect.py` | `ARUCO_TARGET_ID = 0` (전면 마커) |
| `scripts/c270_handeye_capture.py` | `DICT_4X4_50` 하드코딩 |
| `scripts/examples/example_aruco_detection.py` | `DICT_4X4_50` |
| `ros2_ws/src/vision/vision/marker_scan_node.py` | `config/vision.yaml` 경유 |
