# ============================================================
# top_view_seg 학습 스크립트 — Google Colab (T4 GPU)
# 모델: YOLOv11 Nano Instance Segmentation
# 데이터: Roboflow final-project_segmentation v1 (1,574장)
#
# 셀 단위로 순서대로 실행할 것
# API 키 설정: Colab 좌측 🔑 Secrets → ROBOFLOW_API_KEY 추가
# ============================================================


# ── Cell 1: 패키지 설치 ──────────────────────────────────────
# !pip install ultralytics roboflow -q


# ── Cell 2: 데이터셋 다운로드 ────────────────────────────────
# import os
# from roboflow import Roboflow
#
# rf = Roboflow(api_key=os.environ["ROBOFLOW_API_KEY"])  # Colab Secrets
# project = rf.workspace("yeonseop9999-gmail-com").project("final-project_segmentation")
# version = project.version(1)
# dataset = version.download("yolov11")
#
# print("데이터셋 경로:", dataset.location)
# # → /content/final-project_segmentation-1


# ── Cell 3: 학습 ─────────────────────────────────────────────
# from ultralytics import YOLO
#
# model = YOLO("yolo11n-seg.pt")  # Nano segmentation 베이스
#
# model.train(
#     data="/content/final-project_segmentation-1/data.yaml",
#     epochs=100,
#     patience=30,
#     batch=16,
#     imgsz=640,
#     optimizer="auto",
#     lr0=0.01,
#     lrf=0.01,
#     device=0,
#     project="/content/runs/yolo",
#     name="top_view_seg",
#     exist_ok=False,
#     plots=True,
#     verbose=True,
# )


# ── Cell 4: 결과 확인 ────────────────────────────────────────
# import pandas as pd
#
# df = pd.read_csv("/content/runs/yolo/top_view_seg/results.csv")
# cols = [
#     "epoch",
#     "metrics/mAP50(M)",      # segmentation mask mAP
#     "metrics/mAP50-95(M)",
#     "metrics/precision(M)",
#     "metrics/recall(M)",
# ]
# print(df[cols].tail(10))


# ── Cell 5: 가중치 Drive 저장 ────────────────────────────────
# from google.colab import drive
# import shutil
#
# drive.mount("/content/drive")
#
# src = "/content/runs/yolo/top_view_seg/weights/best.pt"
# dst = "/content/drive/MyDrive/top_view_v3-seg-yolo11n-best.pt"
# shutil.copy(src, dst)
# print(f"Drive 저장 완료: {dst}")
#
# # ── Drive 저장 후 할 일 ──────────────────────────────────────
# # 1. Drive 공유 링크 복사 (링크가 있는 모든 사용자 → 보기)
# # 2. 아래 파일의 drive_url 항목에 링크 붙여넣기:
# #    ros2_ws/src/vision/model_library/top_view_model/v3-seg/model_info.yaml
