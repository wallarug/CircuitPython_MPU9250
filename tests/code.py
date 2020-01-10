###  Comparison between drivers
##
import time
import board
import busio

### Adafruit MPU6050 (similar chip)
import adafruit_mpu6050

### RM Forked Driver
from robohat_mpu9250.mpu9250 import MPU9250 as RM9250
from robohat_mpu9250.mpu6500 import MPU6500 as RM6500
from robohat_mpu9250.ak8963 import AK8963 as RM8963

### RM CircuitPython Driver
import roboticsmasters_mpu6500
import roboticsmasters_mpu9250

### i2c
i2c = busio.I2C(board.SCL, board.SDA)

### adafruit driver
mpu = adafruit_mpu6050.MPU6050(i2c, address=0x69)
##
### RM Forked
#rm_mpu = RM6500(i2c, address=0x69)
#rm_ak = RM8963(i2c)
#sensor = RM9250(rm_mpu, rm_ak)

### New Driver
#nmpu = roboticsmasters_mpu6500.MPU6500(i2c, address=0x69)
npmu = roboticsmasters_mpu9250.MPU9250(i2c)

time.sleep(1)

##while True:
##    print("=============")
##    print("Acceleration:")
##    print("X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(mpu.acceleration))
##    print("X:{0:0.2f}, Y: {1:0.2f}, Z: {2:0.2f} m/s^2".format(*sensor.read_acceleration()))
##    print("X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(nmpu.acceleration))
##    print("Gyro:")
##    print("X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(mpu.gyro))
##    print("X:{0:0.2f}, Y: {1:0.2f}, Z: {2:0.2f} degrees/s".format(*sensor.read_gyro()))
##    print("X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(nmpu.gyro))
##    print("Temperature:")
##    print("%.2f C"%mpu.temperature)
##    print("%.2f C"%sensor.read_temperature())
##    print("%.2f C"%nmpu.temperature)
##    print("")
##    time.sleep(2)

 
while not i2c.try_lock():
    pass
 
while True:
    print("I2C addresses found:", [hex(device_address)
                                   for device_address in i2c.scan()])
    time.sleep(2)
