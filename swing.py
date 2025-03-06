import Jetson.GPIO as GPIO
import time


AIN1 = 15
AIN2 = 32
PWM_FREQ = 50

MAX_SIZE = 1000
MIN_SIZE = 1

GPIO.setmode(GPIO.BOARD)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)

pwm = GPIO.PWM(AIN1, PWM_FREQ)
pwm.start(0)

GPIO.output(AIN2, GPIO.LOW)

