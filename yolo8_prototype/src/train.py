#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Train a 2-class YOLOv8n, export INT8-ONNX, and compile to .rknn
for Orange Pi 5 / RK3588 NPU.

依赖：
    pip install ultralytics==8.*  torch<=2.4  \
                rknn-toolkit2==2.3.2  pyyaml  onnx  onnxruntime
    # 其中 torch<=2.4 是为避免 PyTorch 2.6+ 的 weights_only 新安全限制
"""

from pathlib import Path
import random, yaml
from ultralytics import YOLO
from rknn.api import RKNN

# ───────────── ① 全局配置 ─────────────
DATA_YAML = "/home/inubashiri/03_DART/src/dataset/data.yaml"
MODEL_CFG = "yolov8n.yaml"  # 两类任务用 n 版最快
IMG_SIZE = 512  # 与部署分辨率保持一致
EPOCHS = 100
BATCH = 32
EXP_NAME = "exp_base_light_rk3588"
CALIB_N = 200  # 量化校准图片张数
CALIB_TXT = "calib.txt"
# ───────────── ② 训练 ─────────────
print("🚀 开始训练 YOLOv8n ...")
model = YOLO(MODEL_CFG)
_ = model.train(
    data=DATA_YAML,
    imgsz=IMG_SIZE,
    epochs=EPOCHS,
    batch=BATCH,
    name=EXP_NAME,
    workers=8,
)

trainer = model.trainer
save_dir = Path(trainer.save_dir)
best_pt = save_dir / "weights/best.pt"

# ★ 新版取值：匹配任何含 'mAP50' 的键（mAP50, mAP50(B), mAP@0.5…）
final_mAP50 = next(
    (v for k, v in trainer.metrics.items() if "mAP50" in k or "mAP@0.5" in k), None
)

if final_mAP50 is not None:
    print(f"   最终 mAP50 = {final_mAP50:.3f}")
else:
    print("⚠️  未获取到 mAP50（键不存在）")

print(f"🎯 训练完成，best.pt = {best_pt}")
