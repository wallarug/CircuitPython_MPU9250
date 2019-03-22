"""
Test data aquisition from the sensors in a loop.
"""

import time
import mpu6500
import ak8963

interval = 1 / 20 # [s]
n_iterations = 100

imu = mpu6500.MPU6500()
mag = ak8963.AK8963()

t_start_loop = time.time()
for _ in range(n_iterations):
    t_start = time.time()
    acc = imu.read_acceleration()
    rot = imu.read_gyro()
    temp = imu.read_temperature()
    t_imu_end = time.time()
    mfield = mag.read_magnetic()
    t_mag_end = t_end = time.time()

    print("Acceleration:", acc)
    print("Rotation    :", rot)
    print("Temperature :", temp)
    print("Duration IMU:", t_imu_end - t_start)
    print("Magnetic    :", mfield)
    print("Duration MAG:", t_mag_end - t_imu_end)
    print()

    t_end = time.time()
    remaining = max(0, interval - (t_end - t_start))
    time.sleep(remaining)

t_end_loop = time.time()
t_total = t_end_loop - t_start_loop
print("Total time       :", t_total)
print("Average loop time:", t_total / n_iterations)
print("Loop frequency   :", n_iterations / t_total)


