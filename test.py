import time
import mpu6500
import ak8963

interval = 0.5

imu = mpu6500.MPU6500()
mag = ak8963.AK8963()

for _ in range(100):
    t_start = time.time()
    acc = imu.acceleration
    rot = imu.gyro
    temp = imu.temperature
    t_imu_end = time.time()
    mfield = mag.magnetic
    t_mag_end = t_end = time.time()

    print("Acceleration:", acc)
    print("Rotation    :", rot)
    print("Temperature :", temp)
    print("Duration IMU:", t_imu_end - t_start)
    print("Magnetic    :")
    print("Duration MAG:", t_mag_end - t_imu_end)
    print()

    t_end = time.time()
    remaining = max(0, interval - (t_end - t_start))
    time.sleep(remaining)

