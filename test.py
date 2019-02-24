import time
import mpu6500

interval = 0.5

imu = mpu6500.MPU6500()

for _ in range(100):
    tstart = time.time()
    acc = imu.acceleration
    rot = imu.gyro
    temp = imu.temperature
    tend = time.time()

    print("Acceleration:", acc)
    print("Rotation    :", rot)
    print("Temperature :", temp)
    print("Duration    :", tend - tstart)
    print()

    tend = time.time()
    remaining = max(0, interval - (tend - tstart))
    time.sleep(remaining)

