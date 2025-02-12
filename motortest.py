import Jetson.GPIO as GPIO
import time

motor_pwm_pin = 33 #must be pin 33 for PWM

GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor_pwm_pin, GPIO.OUT)

pwm = GPIO.PWM(motor_pwm_pin, 1000) #1kHz frequency
pwm.start(0)

try:
    while True:
        for speed in range(0, 101, 10): #speed up to 100% duty cycle in increments of 10% every 0.5s
            pwm.ChangeDutyCycle(speed)
            time.sleep(0.5)
        for speed in range(100, -1, -10): #slow down
            pwm.ChangeDutyCycle(speed)
            time.sleep(0.5)
except KeyboardInterrupt: #ends with ctrl+c
    pwm.stop()
    GPIO.cleanup()
