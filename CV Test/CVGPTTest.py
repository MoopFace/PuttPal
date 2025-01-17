from ultralytics import YOLO
import cv2

# Load the YOLOv8m model
model = YOLO('yolov8m.pt')  # Ensure you have downloaded the yolov8m.pt model file

# Set up the video capture (0 for default webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

try:
    while True:
        # Read frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Perform object detection
        results = model(frame)

        # Draw the results on the frame
        annotated_frame = results[0].plot()

        # Display the frame
        cv2.imshow('YOLOv8m Object Detection', annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted by user.")

# Release resources
cap.release()
cv2.destroyAllWindows()