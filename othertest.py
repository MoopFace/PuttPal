import board
import digitalio
import busio

print("fat")

pin = digitalio.DigitalInOut(board.D4)

print("nuts")

i2c = busio.I2C(board.SCL, board.SDA)

print("in")

i2c = busio.I2C(board.SCL_1, board.SDA_1)

print("my mouth")
