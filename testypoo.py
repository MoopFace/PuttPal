import Jetson.GPIO as GPIO
import time

#pins
led_pin = 7
print(GPIO.getmode())  # Should print 'BOARD' (2) if set correctly
print(GPIO.gpio_function(7))  # Should return GPIO.OUT if configured properl

#pin setup
GPIO.setmode(GPIO.BOARD)  # use physical pin numbering instead of GPIO numbering, it's easier
GPIO.setup(led_pin, GPIO.OUT)  # led pin set as output

print("press control c to exit. or don't. i don't care. stop asking me things. GET OUT OF MY HOME!\n")

try: 
    while True:
        GPIO.output(led_pin, GPIO.HIGH)  # turn LED ON
        time.sleep(0.5)
        GPIO.output(led_pin, GPIO.LOW)  # turn LED OFF
        time.sleep(0.5) 
except KeyboardInterrupt:
    print("\nExiting")

finally:
    GPIO.cleanup()
    
