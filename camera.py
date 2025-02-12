import cv2
from ultralytics import YOLO
import time

# Load YOLOv11 model  
model = YOLO('/home/jetson/Documents/test/best(1).pt', task="detect")

# Open the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

# Initialize FPS tracking
prev_time = time.time()

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Failed to load camera.")
        break

    # Calculate FPS
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # Perform detection
    results = model.predict(source=frame, conf=0.7, iou=0.7)

    # Annotate detections on the frame
    for result in results:
        if result.boxes is not None:
            for box in result.boxes:
                # Extract bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                confidence = float(box.conf[0])
                label = result.names[int(box.cls[0])]

                # Draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display FPS on the frame
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # Show the annotated frame
    cv2.imshow("Golf Ball Detection", frame)

    # Break out on 'q' or 'ESC' key press
    if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
        break

# Release 
cap.release()
cv2.destroyAllWindows()