# The MIT License (MIT)
#
# Copyright (c) 2020 Cian Byrne for Robotics Masters Limited
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
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
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`roboticsmasters_mpu9250`
================================================================================

CircuitPython helper library for MPU9250 9-axis IMU


* Author(s): Cian Byrne

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s). Use unordered list & hyperlink rST
   inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

from time import sleep
import adafruit_bus_device.i2c_device as i2c_device
from roboticsmasters_mpu6500 import MPU6500
from roboticsmasters_ak8963 import AK8963

try:
    import struct
except ImportError:
    import ustruct as struct
from micropython import const


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/wallarug/CircuitPython_MPU9250.git"

# pylint: disable=bad-whitespace
_MPU6500_DEFAULT_ADDRESS    = 0x69 # MPU6500 default i2c address
_AK8963_DEFAULT_ADDRESS     = 0x0c # AK8963 default i2c address

# pylint: enable=bad-whitespace

class MPU9250:
    """Driver for the MPU9250 9-DoF IMU.
        :param ~busio.I2C i2c_bus: The I2C bus the MPU9250 is connected to.
        :param address: The I2C slave address of the sensor
    """

    def __init__(self, i2c_bus,
                 mpu_addr=_MPU6500_DEFAULT_ADDRESS,
                 akm_addr=_AK8963_DEFAULT_ADDRESS):

        self._mpu = MPU6500(i2c_bus, mpu_addr)
        self._mpu.start_i2c()
        self._akm = AK8963(i2c_bus, akm_addr)

    @property
    def temperature(self):
        """The current temperature in  º C"""
        return self._mpu.temperature()

    @property
    def acceleration(self):
        """Acceleration X, Y, and Z axis data in m/s^2"""
        return self._mpu.acceleration()

    @property
    def gyro(self):
        """Gyroscope X, Y, and Z axis data in º/s"""
        return self._mpu.gyro()

    @property
    def magnetic(self):
        return self._akm.magnetic()

    

    
    
        




