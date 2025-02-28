import time
import board
import digitalio


led = digitalio.DigitalInOut(board.D18)
led.direction = digitalio.Direction.OUTPUT

while True:
    time.sleep(0.5)
    led.value = True
    time.sleep(0.5)
    led.value = False
