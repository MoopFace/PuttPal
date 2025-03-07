import time
import Jetson.GPIO as GPIO
from adafruit_motorkit import MotorKit
import board

HIT = True

#setup motorkit
kit = MotorKit(i2c=board.I2C())



if (HIT): 
    #setup GPIO
    AIN1 = 32
    AIN2 = 33
    PWM_FREQ = 50

    MAX_SIZE = 1000
    MAX_SIZE = 1

    print("THE MODE IS ", GPIO.getmode())
    print("GPIO.BOARD IS ", GPIO.BOARD)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(AIN1, GPIO.OUT)
    pwm = GPIO.PWM(AIN1, PWM_FREQ)
    pwm.start(0)


    GPIO.output(AIN2, GPIO.LOW)





#motorkit move
kit.motor1.throttle=(0.5)
time.sleep(2)
kit.motor1.throttle=(0)


if (HIT):
    #hitter swing
    pwm.ChangeDutyCycle(60)
    time.sleep(1)
    pwm.ChangeDutyCycle(0)


GPIO.cleanup()
