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
_AK8963_DEFAULT_ADDRESS    = 0x0c # AK8963 default i2c address
_AK8963_DEVICE_ID          = 0x48 # MPU9250 WHO_AM_I value

_AK8963_WIA                = 0x00 # Device ID register
_AK8963_INFO               = 0x01 # Device Information register
_AK8963_ST1                = 0x02 # Status register 1
_AK8963_MAG_OUT            = 0x03 # base address for sensor data reads
_AK8963_HXL                = 0x03 # 
_AK8963_HXH                = 0x04
_AK8963_HYL                = 0x05
_AK8963_HYH                = 0x06
_AK8963_HZL                = 0x07
_AK8963_HZH                = 0x08
_AK8963_ST2                = 0x09
_AK8963_CNTL1              = 0x0A
_AK8963_ADJUST             = 0x10 # base address for sensor adjust reads
_AK8963_ASAX               = 0x10
_AK8963_ASAY               = 0x11
_AK8963_ASAZ               = 0x12


class Sensitivity:
    """Allowed values for `range`.

    - ``Rate.CYCLE_1_25_HZ``
    - ``Rate.CYCLE_5_HZ``
    - ``Rate.CYCLE_20_HZ``
    - ``Rate.CYCLE_40_HZ``
    """
    SENSE_14BIT = 0
    SENSE_16BIT = 1

class Mode:
    """Allowed values for `mode` setting

    - ``Mode.MODE_POWERDOWN``
    - ``Mode.MODE_SINGLE``
    - ``Mode.MODE_CONT1``
    - ``Mode.MODE_CONT2``
    - ``Mode.MODE_EXT_TRIG``
    - ``Mode.MODE_SELFTEST``
    - ``Mode.MODE_FUSE``

    """
    POWERDOWN = 0b0000
    MEASURE_SINGLE = 0b0001
    MEASURE_8HZ = 0b0010 # 8 Hz
    EXT_TRIG = 0b0100
    MEASURE_100HZ =  0b0110 # 100 Hz
    SELFTEST = 0b1000
    FUSE = 0b1111
    

class AK8963:
    """Driver for the AK8963 magnetometer.
        :param ~busio.I2C i2c_bus: The I2C bus the AK8963 is connected to.
        :param address: The I2C slave address of the sensor
    """
    def __init__(self, i2c_bus, address=_AK8963_DEFAULT_ADDRESS):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        if self._device_id != _AK8963_DEVICE_ID:
            raise RuntimeError("Failed to find AKM8963 - check your wiring!")

        self.reset()

        _mode = Mode.FUSE
        raw_adjustment = self._raw_adjustment_data
        _mode = Mode.POWERDOWN
        sleep(100e-6)

        asax = raw_adjustment[0][0]
        asay = raw_adjustment[1][0]
        asaz = raw_adjustment[2][0]

        self._offset = (0,0,0)
        self._scale = (1,1,1)
        self._adjustment = (
            ((0.5 * (asax - 128)) / 128) + 1,
            ((0.5 * (asay - 128)) / 128) + 1,
            ((0.5 * (asaz - 128)) / 128) + 1
        )

        self.start()
        
  
    def reset(self):
        """Reinitialize the sensor"""
        self._reset = True
        while self._reset is True:
            sleep(0.001)
        sleep(0.100)

    def start(self):
        """Start Up and Initalise the sensor"""
        _mag_range = Sensitivity.SENSE_16BIT
        _mode = Mode.MEASURE_8HZ
    

    _device_id = ROUnaryStruct(_AK8963_WIA, ">B")
    _reset = RWBit(_AK8963_ST2, 0, 1)

    _raw_magnet_data = StructArray(_AK8963_MAG_OUT, "<h", 3)
    _raw_adjustment_data = StructArray(_AK8963_ADJUST, ">b", 3)

    _mode = RWBits(4, _AK8963_CNTL1, 0)
    _mag_range = RWBit(_AK8963_CNTL1, 4, 1)
    
    _status = ROUnaryStruct(_AK8963_ST2, ">B")


    @property
    def magnetic(self):
        """The magnetometer X, Y, Z axis values as a 3-tuple of
        micro-Tesla (uT) values.
        """
        raw_data = self._raw_magnet_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        """
        X, Y, Z axis micro-Tesla (uT) as floats.
        """
        self._status # Enable updating readings again

        # Apply factory axial sensitivy adjustments
        raw_x *= self._adjustment[0]
        raw_y *= self._adjustment[1]
        raw_z *= self._adjustment[2]

        # Apply output scale determined in constructor
        mag_range = self._mag_range
        mag_scale = 1
        if mag_range == Sensitivity.SENSE_16BIT:
            mag_scale = 0.15
        if mag_range == Sensitivity.SENSE_14BIT:
            mag_scale = 0.6
        
        raw_x *= mag_scale
        raw_y *= mag_scale
        raw_z *= mag_scale

        # Apply hard iron ie. offset bias from calibration
        raw_x -= self._offset[0]
        raw_y -= self._offset[1]
        raw_z -= self._offset[2]

        # Apply soft iron ie. scale bias from calibration
        raw_x *= self._scale[0]
        raw_y *= self._scale[1]
        raw_z *= self._scale[2]

        return (raw_x, raw_y, raw_z)


    def calibrate(self, count=256, delay=0.200):
        """
        Calibrate the magnetometer.

        The magnetometer needs to be turned in alll possible directions
        during the callibration process. Ideally each axis would once 
        line up with the magnetic field.

        count: int
            Number of magnetometer readings that are taken for the calibration.
        
        delay: float
            Delay between the magntometer readings in seconds.
        """
        self._offset = (0, 0, 0)
        self._scale = (1, 1, 1)

        reading = self.magnetic
        minx = maxx = reading[0]
        miny = maxy = reading[1]
        minz = maxz = reading[2]

        while count:
            sleep(delay)
            reading = self.read_magnetic()
            minx = min(minx, reading[0])
            maxx = max(maxx, reading[0])
            miny = min(miny, reading[1])
            maxy = max(maxy, reading[1])
            minz = min(minz, reading[2])
            maxz = max(maxz, reading[2])
            count -= 1

        # Hard iron correction
        offset_x = (maxx + minx) / 2
        offset_y = (maxy + miny) / 2
        offset_z = (maxz + minz) / 2

        self._offset = (offset_x, offset_y, offset_z)

        # Soft iron correction
        avg_delta_x = (maxx - minx) / 2
        avg_delta_y = (maxy - miny) / 2
        avg_delta_z = (maxz - minz) / 2

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

        scale_x = avg_delta / avg_delta_x
        scale_y = avg_delta / avg_delta_y
        scale_z = avg_delta / avg_delta_z

        self._scale = (scale_x, scale_y, scale_z)
    
    
