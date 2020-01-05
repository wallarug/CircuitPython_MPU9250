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
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_register.i2c_struct_array import StructArray
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
import adafruit_bus_device.i2c_device as i2c_device

try:
    import struct
except ImportError:
    import ustruct as struct
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/wallarug/CircuitPython_MPU9250.git"

# pylint: disable=bad-whitespace
_AK8963_DEFAULT_ADDRESS    = 0x69 # AK8963 default i2c address
_AK8963_DEVICE_ID          = 0x48 # MPU9250 WHO_AM_I value

_AK8963_WIA                = 0x00 # Device ID register
_AK8963_INFO               = 0x01 # Device Information register
_AK8963_ST1                = 0x02 # Status register 1
_AK8963_MAG_OUT            = 0x08 # base address for sensor data reads
_AK8963_HXL                = 0x03 # 
_AK8963_HXH                = 0x04
_AK8963_HYL                = 0x05
_AK8963_HYH                = 0x06
_AK8963_HZL                = 0x07
_AK8963_HZH                = 0x08
_AK8963_ST2                = 0x09
_AK8963_CNTL1              = 0x0A
_AK8963_ASAX               = 0x10
_AK8963_ASAY               = 0x11
_AK8963_ASAZ               = 0x12


class AK8963:
    """Driver for the AK8963 magnetometer.
        :param ~busio.I2C i2c_bus: The I2C bus the AK8963 is connected to.
        :param address: The I2C slave address of the sensor
    """
    def __init__(self, i2c_bus, address=_AK8963_DEFAULT_ADDRESS):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        if self._device_id != _AK8963_DEVICE_ID:
            raise RuntimeError("Failed to find MPU6500 - check your wiring!")

        self.reset()



    _device_id = ROUnaryStruct(_AK8963_WIA, ">B")

    _raw_magnet_data = StructArray(_AK8963_MAG_OUT, "<h", 3)

    _fuse_rom = RWBits(2, _AK8963_CNTL1, 3)
    _power_down = RWBits(2, _AK8963_CNTL1, 3)
    
