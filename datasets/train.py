from ultralytics import YOLO

# yolo detect train \
# model=yolov8n.pt \
# data=data.yaml \
# epochs=100 \
# freeze=10 \
# lr0=0.001 \
# mixup=0.2 \
# weight_decay=0.0005 \
# patience=20

# Load the nano model
model = YOLO('yolov8n.pt')

# Train the model
results = model.train(data='/media/nilum/New_Volume/01.projects/Final-Project-Robotics-and-Automation/datasets/Cube-Detection-Dataset-3/data.yaml', epochs=100, imgsz=640, device='0', lr0=0.001) # Use device='cpu' if no GPU

# Evaluate the model on the test set
model = YOLO("runs/detect/train/weights/best.pt")

metrics = model.val(data="/media/nilum/New_Volume/01.projects/Final-Project-Robotics-and-Automation/datasets/Cube-Detection-Dataset-3/data.yaml", split="test")

print(metrics)