# 캘리브레이션 참고 자료

> **임시 참고 디렉토리** — 마커 배치·캘리브레이션 결과·검증 절차를 한 곳에서 관리.
> 샘플 이미지 폴더(`scripts/samples_c270_*/`)는 gitignore 처리되어 있어 로컬 전용.

---

## 마커 구성 (DICT_4X4_50)

인쇄 파일: `scripts/multi_item/markers/aruco_markers.pdf`

| ID | 파일 | 물리 배치 위치 | 용도 |
|----|------|---------------|------|
| 0 | `marker_0_ID0_ceiling.png` | 천장 고정 | 탑뷰 D455f 캘리브레이션 / C270 eye-in-hand 캘리브레이션 |
| 1 | `marker_1_ID1_drawer_layer0.png` | 공구함 서랍 레이어 0 | 탑뷰 공구함 위치 인식 |
| 2 | `marker_2_ID2_drawer_layer1.png` | 공구함 서랍 레이어 1 | 탑뷰 공구함 위치 인식 |

- 마커 한 변 길이: **5cm** (실측값, 오차 ±1mm 이내 필수)
- 출력 시 `_clean.png` 버전 권장 (테두리 여백 최소화)

---

## 카메라별 캘리브레이션 상태

| 카메라 | 인트린직 | 핸드-아이 | 브랜치 |
|--------|---------|----------|--------|
| RealSense D455f (탑뷰) | 내장 | Point Correspondence 완료 (2.91mm) | `feat/topview-calibration` |
| Logitech C270 (그리퍼) | ✅ 완료 2026-06-09 | 🔄 진행 중 | `feat/handeye-calibration` |

---

## 디렉토리 구조

```
scripts/
├── aruco_markers/          ← 인쇄용 마커 PNG + PDF (git 추적)
├── samples_c270_calib/     ← C270 인트린직 캘리브레이션 체커보드 샘플 21장 (gitignore)
├── samples_c270_handeye/   ← C270 핸드-아이 수집 이미지 (gitignore)
├── samples_c270/           ← C270 일반 캡처 (gitignore)
└── calib_ref/              ← 이 디렉토리 — 참고 자료 (git 추적)
    └── c270/
        ├── intrinsic.md    ← 인트린직 결과 + 물리 검증 체크리스트
        └── handeye.md      ← 핸드-아이 수집 절차 + 마커 배치 안내
```
