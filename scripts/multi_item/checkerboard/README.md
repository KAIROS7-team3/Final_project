# 체커보드 (Checkerboard)

C270 인트린직 캘리브레이션(`c270_intrinsic_calib.py`)에 사용하는 패턴.

## 규격

| 항목 | 값 | 출처 |
|------|----|------|
| 내부 코너 수 | **9 × 6** (cols × rows) | `config/c270_camera_info.yaml` → `board` |
| 한 칸 크기 | **25mm** | `config/c270_camera_info.yaml` → `square_m: 0.025` |
| 전체 크기 (참고) | 약 250mm × 175mm (10×7칸 기준) | — |

> 내부 코너 기준이므로 실제 격자 칸 수는 10×7.

## 인쇄 방법

1. OpenCV 체커보드 생성 스크립트 사용:
   ```bash
   python3 - <<'EOF'
   import cv2, numpy as np
   board = np.zeros((175*4, 250*4), np.uint8)
   sq = 25*4
   for r in range(7):
       for c in range(10):
           if (r + c) % 2 == 0:
               board[r*sq:(r+1)*sq, c*sq:(c+1)*sq] = 255
   cv2.imwrite('scripts/multi_item/checkerboard/checkerboard_9x6_25mm.png', board)
   EOF
   ```
2. 출력 후 한 칸 실측 필수 — **25mm ± 0.5mm** 이내
3. 딱딱한 판에 부착 (구겨지면 캘리브 오차 증가)
4. 라미네이팅 권장 (무광)

## 캘리브레이션 결과 위치

```
config/c270_camera_info.yaml   ← 인트린직 결과 (fx, fy, cx, cy, 왜곡계수)
scripts/samples_c270_calib/    ← 캘리브에 사용한 샘플 이미지 (gitignore, 로컬 전용)
scripts/calib_ref/c270/intrinsic.md  ← 결과 기록 + 물리 검증 체크리스트
```

## 참조하는 코드

| 파일 | 참조 방식 |
|------|----------|
| `scripts/c270_intrinsic_calib.py` | `BOARD_COLS=9`, `BOARD_ROWS=6`, `SQUARE_M=0.025` |
| `config/c270_camera_info.yaml` | `board.cols`, `board.rows`, `board.square_m` |
