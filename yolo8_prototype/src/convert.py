from ultralytics import YOLO

# Load a model
model = YOLO(
    "/home/inubashiri/03_DART/src/runs/detect/exp_base_light_rk3588/weights/best.pt"
)  # load a model
# Export the model
model.export(format="onnx", imgsz=[480, 640], opset=12)  # 导出一定不要修改这里参数
