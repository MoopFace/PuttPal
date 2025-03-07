import time
#import Jetson.GPIO as GPIO
from adafruit_motorkit import MotorKit
import board
import pwmio
import digitalio


#OLD_HIT = False
NEW_HIT = True

if (NEW_HIT):
    print("new hit setup")
    AIN1 = 32
    AIN2 = 33
    PWM_FREQ = 50
    
    directionPin = digitalio.DigitalInOut(board.D13) #should be pin 33 or D13
    directionPin.direction = digitalio.Direction.OUTPUT 
    directionPin.value = False
    
    pwm = pwmio.PWMOut(board.D12, frequency=PWM_FREQ, duty_cycle=0)




#if (OLD_HIT): 
#    #setup GPIO
#    AIN1 = 32
#    AIN2 = 33
#    PWM_FREQ = 50
#
#    print("THE MODE IS ", GPIO.getmode())
#    print("GPIO.BOARD IS ", GPIO.BOARD)
#
#    GPIO.setmode(GPIO.BOARD)
#    GPIO.setup(AIN1, GPIO.OUT)
#    pwm = GPIO.PWM(AIN1, PWM_FREQ)
#    pwm.start(0)
#
#
#    GPIO.output(AIN2, GPIO.LOW)

#setup motorkit
kit = MotorKit(i2c=board.I2C())





#motorkit move
kit.motor1.throttle=(0.5)
time.sleep(2)
kit.motor1.throttle=(0)

if (NEW_HIT):
    print("new hit hitting")
    pwm.duty_cycle = int(65535 * 0.75)
    time.sleep(2)
    pwm.duty_cycle = 0


#if (OLD_HIT):
#    #hitter swing
#    pwm.ChangeDutyCycle(60)
#    time.sleep(1)
#    pwm.ChangeDutyCycle(0)


#GPIO.cleanup()
