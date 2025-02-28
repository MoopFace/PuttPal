import board
import digitalio
import busio

print("hell blinka")

pin = digitalio.DigitalInOut(board.D4)
print("digital io ok")

i2c = busio.I2C(board.SCL, board.SDA)
print("i2c 1 ok")

i2c = busio.I2C(board.SCL_1, board.SDA_1)
print("I2C 2 ok")

print("done")
