
import time
import cv2
from ultralytics import YOLO
import funcs as f



#800 by 450

kit = f.kit
pwm = f.pwm




dutyCycle = int(65535 * 0.8)
speed = 0.85
turntime = 0.05 #0.05 works
movetime = 0.1
searchTime = 0.15
stareTime = 0.1
delay = 0
hittingTime = 3
strafeLeftTime = 1
strafeForwardTime = 1

nextTime = 0

def hit_sequence():
    print("HIT DA BALLS")
    set_strafe_left()
    move()
    time.sleep(strafeLeftTime)
    set_stop()
    move()
    #pwm.ChangeDutyCycle(dutyCycle)
    pwm.duty_cycle = dutyCycle
    set_forward()
    move()
    time.sleep(hittingTime)
    set_stop()
    move()
    #pwm.ChangeDutyCycle(0)
    pwm.duty_cycle = 0
 

  

# Load YOLOv11 model for golf ball detection
model = YOLO('/home/jetson/PuttPal/best.pt', task="detect") # Change the path to the best.pt file

# Open the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

# Initialize FPS tracking
prev_time = time.time()


#stop moving when program starts
f.set_stop()
f.move()
nextTime = 0
moving = 0

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
    results = model.predict(source=frame, conf=0.5, iou=0.5)

    #stop moving if there is nothing and enough time has passed
    if(time.time() > nextTime):
        if (moving == 1):
            f.set_stop()
            f.move()
            moving = 0
#            print("stopped moving")
   
 #   print(len(results[0].boxes))
 #   if 'results' not in locals():
 #           print("nothing detected")
 #           #look for ball
 #           set_left()
 #           move()
 #           nextTime = searchTime
 #           moving = 1
 #           #no ball detected
 #           #just turn in a circle



    # Annotate detections on the frame
    for result in results:
        if (len(result.boxes) == 0):
            print("nothing detected, turning left")
            time.sleep(stareTime)
            f.set_left()
            f.move()
            moving = 1
            nextTime = searchTime

        if result.boxes is not None:

            maxconf = 0
            for box in result.boxes:
                if (maxconf < box.conf[0]):
                    maxconf = box.conf[0]


            for box in result.boxes:
                # Extract bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                confidence = float(box.conf[0])
                label = result.names[int(box.cls[0])] 

                #print(result.names)
                xavg = (x1 + x2) / 2
                yavg = (y1 + y2) / 2
                screen_width = 800
                screen_height = 450
                center = screen_width/2
                x_error = xavg - center
                y_error = yavg - (screen_height/2)
                y_set_point = screen_height * 0.9 
                zone_width = 25
                
                toWait = (turntime/center) * abs(x_error)
        

                if (box.conf[0] == maxconf):
                    if (int(box.cls[0]) == 0):
                        if (time.time() > nextTime):

                        
                            if (x_error > zone_width):
#                                print ("ball found: turn right")
                                #turn right
                                f.set_right()
                                f.move()
                                nextTime = time.time() + toWait
                                moving = 1
#                                print("next time is ", nextTime, " movetime is", movetime, "moving is", moving)
                            

                            elif (x_error < -zone_width):
#                                print("ball found: turn left")
                                #turn left
                                f.set_left()
                                f.move()
                                nextTime = time.time() + toWait
                                moving = 1
#                                print("next time is ", nextTime, " movetime is", movetime, "moving is", moving)
                                

                            elif (-zone_width < x_error < zone_width):
                                # if the ball is centered
                                
                                if (yavg < y_set_point):
                                    #go forwards
                                    f.set_forward()
                                    f.move()
                                    nextTime = time.time() + movetime
                                    moving = 1
#                                    print("next time is ", nextTime, " movetime is", movetime, "moving is", moving)
                                else:
                                    f.hit_sequence()

                        else:
                            print("not enough time has passed")
                            print("current time=", time.time(), " nextTime=", nextTime)


                # Draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}, x={xavg}, y={yavg}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display FPS on the frame
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # Show the annotated frame
#    cv2.imshow("Golf Ball Detection", frame)

    # Break on 'q' or 'ESC' key press
    if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
        break

# Release resources
cap.release()
cv2.destroyAllWindows()

