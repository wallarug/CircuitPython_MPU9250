# Python MPU-9250 (MPU-6500 + AK8963) I2C driver

MPU-9250 is a System in Package (SiP) which combines two chips: MPU-6500 which contains 3-axis gyroscope and 3-axis accelerometer and an AK8963 which is a 3-axis digital compass.

This library communicates with these sensors over I2C. It is written for 
CircuitPython.

## Usage

Simple test with never ending loop.

```python
import time
from mpu9250 import MPU9250

sensor = MPU9250()

print("MPU9250 id: " + hex(sensor.whoami))

while True:
    print(sensor.read_acceleration())
    print(sensor.read_gyro())
    print(sensor.read_magnetic())

    time.sleep(1000)
```

The library returns a 3-tuple of X, Y, Z axis values for either acceleration, gyroscope and magnetometer ie compass. Default units are `m/s^2`, `rad/s` and `uT`. It is possible to also get acceleration values in `g` and gyro values `deg/s`. See the example below. 

Note that both the MPU6500 and the AK8963 drivers are available as separate classes. MPU9250 is actually a composite of those two.

## Magnetometer Calibration

For real life applications you should almost always [calibrate the magnetometer](https://appelsiini.net/2018/calibrate-magnetometer/). The AK8963 driver supports both hard and soft iron correction. Calibration function takes two parameters: `count` is the number of samples to collect and `delay` is the delay in milliseconds between the samples.

With the default values of `256` and `200` calibration takes approximately one minute. While calibration function is running the sensor should be rotated multiple times around each axis.

```python
from mpu9250 import MPU9250
from ak8963 import AK8963

ak8963 = AK8963()
offset, scale = ak8963.calibrate(count=256, delay=200)

sensor = MPU9250(ak8963=ak8963)
```

After finishing calibration the `calibrate()` method also returns tuples for both hard iron `offset` and soft iron `scale`. To avoid calibrating after each startup it would make sense to store these values in NVRAM or config file and pass them to the AK8963 constructor. Below example only illustrates how to use the constructor.

```python
from mpu9250 import MPU9250
from ak8963 import AK8963

ak8963 = AK8963(
    offset=(-136.8931640625, -160.482421875, 59.02880859375),
    scale=(1.18437220840483, 0.923895823933424, 0.931707933618979)
)
sensor = MPU9250(i2c, ak8963=ak8963)
```


## License

The MIT License (MIT). Please see [License File](LICENSE.txt) for more information.
