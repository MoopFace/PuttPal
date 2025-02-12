import curses
import time
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
speed = 0.1

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

def set_left():
    M[1] += -speed
    M[2] += speed
    M[3] += speed
    M[4] += -speed


def set_right():
    M[1] += speed
    M[2] += -speed
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


def move():
    kit.motor1.throttle=norm(M[1])
    kit.motor2.throttle=norm(-M[2])
    kit.motor3.throttle=norm(-M[3])
    kit.motor4.throttle=norm(M[4])

#forward()
#time.sleep(1)
#backward()
#time.sleep(1)
#stop()


def main(stdscr):
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.clear()
    stdscr.addstr("Press arrow keys (ESC to exit)\n")

    pressed_keys = set()
    key_map = {
        curses.KEY_UP: "UP",
        curses.KEY_DOWN: "DOWN",
        curses.KEY_LEFT: "LEFT",
        curses.KEY_RIGHT: "RIGHT"
    }

    while True:
        key = stdscr.getch()
        
        if key == 27:  # ESC key to exit
            set_stop()
            move()
            break
        
        if key in key_map:
            pressed_keys.add(key_map[key])
        elif key == ord(' '):  # Space to reset
            pressed_keys.clear()
            set_stop()
            move()
        
        stdscr.clear()
        stdscr.addstr("Press arrow keys (ESC to exit)\n")
        stdscr.addstr("Current combination: " + ", ".join(sorted(pressed_keys)) + "\n")
       
        if "UP" in pressed_keys:
            set_forward()
        if "DOWN" in pressed_keys:
            set_backward()
        if "LEFT" in pressed_keys:
            set_left()
        if "RIGHT" in pressed_keys:
            set_right()

        move()
        #time.sleep(0.1)

        # Remove key when released (optional, depends on terminal behavior)
#        curses.napms(100)
#        pressed_keys.clear()

if __name__ == "__main__":
    curses.wrapper(main)

