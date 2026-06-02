# ============================================================
# top_view_v3 학습 스크립트 — Google Colab (T4 GPU)
# 셀 단위로 순서대로 실행할 것
# ============================================================

# ── Cell 1: 패키지 설치 ──────────────────────────────────────
# !pip install ultralytics roboflow -q

# ── Cell 2: 데이터셋 다운로드 (Roboflow v2) ──────────────────
# !pip install roboflow
#
# from roboflow import Roboflow
# rf = Roboflow(api_key="TRSBojryEBlxSSsZyv3f")
# project = rf.workspace("yeonseop9999-gmail-com").project("final-project-kir4p")
# version = project.version(2)
# dataset = version.download("yolov11")

# ── Cell 3: 학습 ─────────────────────────────────────────────
from ultralytics import YOLO

model = YOLO("yolo11s.pt")

model.train(
    data="/content/final-project-kir4p-2/data.yaml",
    epochs=200,
    patience=50,
    batch=16,
    imgsz=640,
    optimizer="auto",
    lr0=0.01,
    lrf=0.01,
    device=0,
    project="/content/runs/yolo",
    name="top_view_v3",
    exist_ok=False,
    plots=True,
    verbose=True,
)

# ── Cell 4: 결과 확인 ────────────────────────────────────────
# import pandas as pd
# df = pd.read_csv("/content/runs/yolo/top_view_v3/results.csv")
# print(df[["epoch", "metrics/mAP50(B)", "metrics/mAP50-95(B)"]].tail(10))

# ── Cell 5: 가중치 Drive 저장 ────────────────────────────────
# from google.colab import drive
# import shutil
# drive.mount("/content/drive")
# shutil.copy(
#     "/content/runs/yolo/top_view_v3/weights/best.pt",
#     "/content/drive/MyDrive/top_view_v3_best.pt"
# )
# print("저장 완료")
