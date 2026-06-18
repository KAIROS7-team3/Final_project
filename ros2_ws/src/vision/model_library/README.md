# Model Library

YOLO 모델을 카메라별·버전별로 관리하는 디렉토리.

모델 파일(`.pt`)과 학습 결과물은 용량 초과로 git 제외 — Google Drive에서 다운로드 후 각 버전 폴더의 `weights/`에 배치한다.

---

## 디렉토리 구조

```
model_library/
├── README.md                        ← 이 파일 (git 추적)
├── top_view_model/                  ← 탑뷰 D455f 모델
│   ├── v1/
│   │   ├── model_info.yaml          ← 버전 정보·mAP·클래스·Drive 링크 (git 추적)
│   │   ├── weights/                 ← best.pt 로컬 전용 (gitignore)
│   │   └── results/                 ← 학습 결과물 로컬 전용 (gitignore)
│   └── v2/                          ← 현재 운용 버전
│       ├── model_info.yaml
│       ├── weights/
│       └── results/
└── gripper_view_model/              ← 그리퍼 캠 C270 모델 (추후 추가)
```

---

## 모델 다운로드

1. 사용할 버전의 `model_info.yaml`에서 `drive_url` 확인
2. Google Drive 링크 접속 → 다운로드
3. 해당 버전의 `weights/best.pt`로 저장

```bash
# gdown 사용 예시
pip install gdown

# top_view 모델 (drive_url은 해당 model_info.yaml 확인)
gdown "<drive_url>" -O ros2_ws/src/vision/model_library/top_view_model/v3-2/weights/best.pt

# gripper_view 모델 (C270, v1)
gdown "https://drive.google.com/file/d/1R6OaOKuF2wCM6QxsFsRnNgbGIXPKvB9N/view?usp=sharing" \
    --fuzzy -O ros2_ws/src/vision/model_library/gripper_model/v1/weights/best.pt
```

---

## 현재 운용 버전

| 카메라 | 버전 | 태스크 | mAP@0.5 | 상태 | Drive 링크 |
|--------|------|--------|---------|------|------------|
| top_view (D455f) | v3-2 | Detection | 0.952 | ✅ 운용 중 | `top_view_model/v3-2/model_info.yaml` 참조 |
| top_view (D455f) | v3-seg | Segmentation | 0.977 | 🆕 검증 대기 | `top_view_model/v3-seg/model_info.yaml` 참조 |
| gripper_view (C270) | v1 | Segmentation | — | ✅ 운용 중 | [Google Drive](https://drive.google.com/file/d/1R6OaOKuF2wCM6QxsFsRnNgbGIXPKvB9N/view?usp=sharing) |

운용 버전은 각 `model_info.yaml`의 `active: true` 항목 기준.  
`config/vision.yaml`의 `top_view_model_path`가 실제 사용 경로.

### v3-seg 전환 절차

v3-seg 검증 완료 후 아래 절차로 운용 버전 교체:

1. `top_view_model/v3-seg/model_info.yaml` → `active: true`
2. `top_view_model/v3-2/model_info.yaml` → `active: false`
3. `config/vision.yaml` → `top_view_model_path` 경로 변경
4. `infer_topview_demo.py`로 실물 공구 검출 확인

---

## 새 버전 추가 절차

1. 코랩 학습 완료 → `best.pt` 다운로드
2. `top_view_model/vN/` 폴더 생성
3. `model_info.yaml` 작성 (이전 버전 참고)
4. Google Drive 업로드 → `drive_url` 기입
5. 이전 버전 `model_info.yaml`의 `active: false` 변경
6. `config/vision.yaml`의 `top_view_model_path` 갱신
7. PR 생성
