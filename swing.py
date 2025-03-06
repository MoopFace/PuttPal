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


try:
    while True:
        pwm.ChangeDutyCycle(80)
        time.sleep(3)
        pwm.ChangeDutyCycle(0)
except KeyboardInterrupt:
    print("    all done")

finally:
    try:
        pwm.stop()
    except Exception as e:
        print(f"Error stopping PWM: {e}")
        time.sleep(0.1)
        GPIO.cleanup()

