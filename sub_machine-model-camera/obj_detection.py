import cv2
import torch
from ultralytics import YOLO
from picamera2 import Picamera2
import os

# Model path
path=(os.path.dirname(os.path.abspath(__file__)))
model = YOLO(f"{path}/detection_v8n_320_fp16.pt")

# Detection
def detect_obj(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Run YOLO5 inference
    results = model(frame)

    # Dict for object count
    class_counts = {
        "assembly" : 0,
        "metal" : 0,
        "plastic" : 0
    }

    # Draw bounding box and label
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0]) # get bounding coordinates
            conf = float(box.conf[0]) # Confidence score

            if conf > 0.8:
                cls = int(box.cls[0]) # Class index
                class_name = model.names[cls]
                label = f"{class_name}: {conf:.2f}"

                if class_name in class_counts:
                    class_counts[class_name] += 1

                # Draw rectange and label
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    
    return frame, class_counts["assembly"], class_counts["metal"], class_counts["plastic"]


