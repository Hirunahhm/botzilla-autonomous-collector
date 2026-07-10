from ultralytics import YOLO
import cv2

# Load your trained model
model = YOLO("/media/nilum/New_Volume/01.projects/Final-Project-Robotics-and-Automation/datasets/runs/detect/train/weights/best.pt")

# Provide your video path here
video_path = "/media/nilum/New_Volume/01.projects/Final-Project-Robotics-and-Automation/datasets/vids/01.avi"
# Open video
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error opening video file")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Run inference
    results = model(frame, conf=0.9)

    # Draw detections
    annotated_frame = results[0].plot()

    # Show output
    cv2.imshow("YOLOv8 Detection", annotated_frame)

    # Press q to exit
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()