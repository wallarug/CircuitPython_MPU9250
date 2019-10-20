# Copyright (c) 2018-2019 Mika Tuupola
# Copyright (c) 2019      Eike Welk
# Copyright (c) 2019      Cian Byrne (wallarug) for Robotics Masters Limited 
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of  this software and associated documentation files (the "Software"), to
# deal in  the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copied of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


"""
Python I2C driver for MPU9250 9-axis motion tracking device
"""

# pylint: disable=import-error
from robohat_mpu9250.mpu6500 import MPU6500
from robohat_mpu9250.ak8963 import AK8963
# pylint: enable=import-error

__version__ = "1.0.1"

class MPU9250:
    """Class which provides interface to MPU9250 9-axis motion tracking device."""
    def __init__(self, mpu6500 = None, ak8963 = None):
        if mpu6500 is None:
            self.mpu6500 = MPU6500()
        else:
            self.mpu6500 = mpu6500

        if ak8963 is None:
            self.ak8963 = AK8963()
        else:
            self.ak8963 = ak8963

    def read_acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis values in m/s^2 as floats. To get values in g
        pass `accel_fs=SF_G` parameter to the MPU6500 constructor.
        """
        return self.mpu6500.read_acceleration()

    def read_gyro(self):
        """
        Gyro measured by the sensor. By default will return a 3-tuple of
        X, Y, Z axis values in rad/s as floats. To get values in deg/s pass
        `gyro_sf=SF_DEG_S` parameter to the MPU6500 constructor.
        """
        return self.mpu6500.read_gyro()

    def read_magnetic(self):
        """
        X, Y, Z axis micro-Tesla (uT) as floats.
        """
        return self.ak8963.read_magnetic()

    def read_temperature(self):
        """
        Temperature measured by the sensor.
        """
        return self.mpu6500.read_temperature()

    def read_whoami(self):
        return self.mpu6500.read_whoami()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
