import cv2
from ultralytics import YOLO
import time

#import curses
import board
#import digitalio
#import busio
#print("BUSSSSS")

from adafruit_motorkit import MotorKit
#i2c_bus = busio.I2C(board.SCL_1, board.SDA_1)
#kit = MotorKit(i2c=i2c_bus, )
#kit = MotorKit()
#kit = MotorKit(i2c=busio.I2C(GP13_I2C2_CLK, GP14_I2C_DAT))
#kit = MotorKit(i2c=busio.I2C(28, 27))

kit = MotorKit(i2c=board.I2C())
#3 and 2 are reversed
#    kit.motor1.throttle=0.5
#    kit.motor2.throttle=-0.5
#    kit.motor3.throttle=-0.5
#    kit.motor4.throttle=0.5
    

#def forward():
#    kit.motor1.throttle=0.5
#    kit.motor2.throttle=-0.5
#    kit.motor3.throttle=-0.5
#    kit.motor4.throttle=0.5


M = [0, 0, 0, 0, 0]
speed = 0.85
turntime = 0.1 #0.05 works
movetime = 0.1
delay = 0

nextTime = 0


def set_forward():
    M[1] += speed
    M[2] += speed
    M[3] += speed
    M[4] += speed


def set_backward():
    M[1] += -speed
    M[2] += -speed
    M[3] += -speed
    M[4] += -speed

def set_right():
    M[1] += -speed
    M[2] += speed
    M[3] += speed
    M[4] += -speed


def set_left():
    M[1] += speed
    M[2] += -speed
    M[3] += -speed
    M[4] += speed

def set_strafe_right():
    M[1] += speed
    M[2] += -speed
    M[3] += speed
    M[4] += -speed

def set_strafe_left():
    M[1] += -speed
    M[2] += speed
    M[3] += -speed
    M[4] += speed



def set_stop():
    M[1] = 0
    M[2] = 0
    M[3] = 0
    M[4] = 0

def norm(val):
    if val > 1:
        return 1
    elif val < -1:
        return -1
    return val


def move():
    kit.motor1.throttle=norm(M[1])
    kit.motor2.throttle=norm(-M[2])
    kit.motor3.throttle=norm(-M[3])
    kit.motor4.throttle=norm(M[4])

#def timeMilis():
#    return (time.time() * 1000)

#def timeYet(deltaSeconds):
#    return (time.time() - deltaSeconds) > lastTime



# Load YOLOv11 model for golf ball detection
model = YOLO('/home/jetson/PuttPal/best.pt', task="detect") # Change the path to the best.pt file

# Open the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

# Initialize FPS tracking
prev_time = time.time()


#movement code
set_stop()
move()
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
            set_stop()
            move()
            moving = 0
            print("stopped moving")





    # Annotate detections on the frame
    for result in results:
        if result.boxes is None:
            print("nothing detected")
            #no ball detected
            #just turn in a circle

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
                center = screen_width/2
                error = xavg - center
                zone_width = 75
                
                toWait = (turntime/center) * abs(error)
        

                if (box.conf[0] == maxconf):
                    if (int(box.cls[0]) == 0):
                        if (time.time() > nextTime):

                        
                            if (error > zone_width):
                                print ("ball found: turn right")
                                #turn right
                                set_right()
                                move()
                                nextTime = time.time() + (turntime) * abs(error) 
                                moving = 1
                                print("next time is ", nextTime, " movetime is", movetime, "moving is", moving)
                            

                            elif (error < -zone_width):
                                print("ball found: turn left")
                                #turn left
                                set_left()
                                move()
                                nextTime = time.time() + (turntime) * abs(error) 
                                moving = 1
                                print("next time is ", nextTime, " movetime is", movetime, "moving is", moving)
                                

                            elif (-zone_width < error < zone_width):
                                #go forwards
                                set_forward()
                                move()
                                nextTime = time.time() + movetime
                                moving = 1
                                print("next time is ", nextTime, " movetime is", movetime, "moving is", moving)
                                print("going forwards")
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

