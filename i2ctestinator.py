import time
import board
#import digitalio
#import busio
#print("BUSSSSS")

from adafruit_motorkit import MotorKit
#i2c_bus = busio.I2C(board.SCL_1, board.SDA_1)
#kit = MotorKit(i2c=i2c_bus, )
#kit = MotorKit()
#kit = MotorKit(i2c=busio.I2C(GP13_I2C2_CLK, GP14_I2C_DAT))
#kit = MotorKit(i2c=busio.I2C(28, 27))

kit = MotorKit(i2c=board.I2C())
# 3 and 2 are reversed

kit.motor1.throttle=0.5
kit.motor2.throttle=-0.5
kit.motor3.throttle=-0.5
kit.motor4.throttle=0.5
