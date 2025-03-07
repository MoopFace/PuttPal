import time
import board
#import Jetson.GPIO as GPIO
import digitalio
import pwmio
from adafruit_motorkit import MotorKit


kit = MotorKit(i2c=board.I2C())


PWM_FREQ = 50
directionPin = digitalio.DigitalInOut(board.D13) #should be pin 33 or D13
directionPin.direction = digitalio.Direction.OUTPUT 
directionPin.value = False

pwm = pwmio.PWMOut(board.D12, frequency=PWM_FREQ, duty_cycle=0)


M = [0, 0, 0, 0, 0]

def set_forward(this_speed):
    M[1] += this_speed
    M[2] += this_speed
    M[3] += this_speed
    M[4] += this_speed


def set_backward(this_speed):
    M[1] += -this_speed
    M[2] += -this_speed
    M[3] += -this_speed
    M[4] += -this_speed

def set_right(this_speed):
    M[1] += -this_speed
    M[2] += this_speed
    M[3] += this_speed
    M[4] += -this_speed


def set_left(this_speed):
    M[1] += this_speed
    M[2] += -this_speed
    M[3] += -this_speed
    M[4] += this_speed

def set_strafe_right(this_speed):
    M[1] += this_speed
    M[2] += -this_speed
    M[3] += this_speed
    M[4] += -this_speed

def set_strafe_left(this_speed):
    M[1] += -this_speed
    M[2] += this_speed
    M[3] += -this_speed
    M[4] += this_speed

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



