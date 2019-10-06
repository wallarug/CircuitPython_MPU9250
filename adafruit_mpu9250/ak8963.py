# Copyright (c) 2018-2019 Mika Tuupola
# Copyright (c) 2019      Eike Welk
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

# https://www.akm.com/akm/en/file/datasheet/AK8963C.pdf


"""
Python I2C driver for AK8963 magnetometer
"""
from time import sleep

__version__ = "0.2.0-a"
__repo__ = "https://github.com/wallarug/circuitpython_mpu9250"

try:
    import struct
except ImportError:
    import ustruct as struct

import adafruit_bus_device.i2c_device as i2c_device
from micropython import const

# pylint: disable=bad-whitespace

_WIA = 0x00
_HXL = 0x03
_HXH = 0x04
_HYL = 0x05
_HYH = 0x06
_HZL = 0x07
_HZH = 0x08
_ST2 = 0x09
_CNTL1 = 0x0a
_ASAX = 0x10
_ASAY = 0x11
_ASAZ = 0x12

_MODE_POWER_DOWN = 0b00000000
MODE_SINGLE_MEASURE = 0b00000001
MODE_CONTINUOUS_MEASURE_1 = 0b00000010 # 8Hz
MODE_CONTINUOUS_MEASURE_2 = 0b00000110 # 100Hz
MODE_EXTERNAL_TRIGGER_MEASURE = 0b00000100
_MODE_SELF_TEST = 0b00001000
_MODE_FUSE_ROM_ACCESS = 0b00001111

OUTPUT_14_BIT = 0b00000000
OUTPUT_16_BIT = 0b00010000

_SO_14BIT = 0.6 # μT per digit when 14bit mode
_SO_16BIT = 0.15 # μT per digit when 16bit mode

class AK8963:
    """Class which provides interface to AK8963 magnetometer."""
    def __init__(
        self, i2c_interface=None, busnum=1, address=0x0c,
        mode=MODE_CONTINUOUS_MEASURE_2, output=OUTPUT_16_BIT,
        offset=(0, 0, 0), scale=(1, 1, 1)
    ):
        self.i2c = i2c_device.I2CDevice(i2c_interface, address)

        self.address = address
        self._offset = offset
        self._scale = scale

        if 0x48 != self.read_whoami():
            raise RuntimeError("AK8963 not found in I2C bus.")

        # Sensitivity adjustment values
        self._write_register_char(_CNTL1, _MODE_FUSE_ROM_ACCESS)
        asax = self._read_register_char(_ASAX)
        asay = self._read_register_char(_ASAY)
        asaz = self._read_register_char(_ASAZ)
        self._write_register_char(_CNTL1, _MODE_POWER_DOWN)

        # Should wait at least 100us before next mode
        time.sleep(100e-6)

        self._adjustment = (
            (0.5 * (asax - 128)) / 128 + 1,
            (0.5 * (asay - 128)) / 128 + 1,
            (0.5 * (asaz - 128)) / 128 + 1
        )

        # Power on
        self._write_register_char(_CNTL1, (mode | output))

        if output is OUTPUT_16_BIT:
            self._so = _SO_16BIT
        else:
            self._so = _SO_14BIT

    def read_magnetic(self):
        """
        X, Y, Z axis micro-Tesla (uT) as floats.
        """
        xyz = list(self._read_register_three_shorts(_HXL))
        self._read_register_char(_ST2) # Enable updating readings again

        # Apply factory axial sensitivy adjustments
        xyz[0] *= self._adjustment[0]
        xyz[1] *= self._adjustment[1]
        xyz[2] *= self._adjustment[2]

        # Apply output scale determined in constructor
        so = self._so
        xyz[0] *= so
        xyz[1] *= so
        xyz[2] *= so

        # Apply hard iron ie. offset bias from calibration
        xyz[0] -= self._offset[0]
        xyz[1] -= self._offset[1]
        xyz[2] -= self._offset[2]

        # Apply soft iron ie. scale bias from calibration
        xyz[0] *= self._scale[0]
        xyz[1] *= self._scale[1]
        xyz[2] *= self._scale[2]

        return tuple(xyz)

    def read_whoami(self):
        """ Value of the whoami register. """
        return self._read_register_char(_WIA)

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

        reading = self.read_magnetic()
        minx = maxx = reading[0]
        miny = maxy = reading[1]
        minz = maxz = reading[2]

        while count:
            time.sleep(delay)
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

        return self._offset, self._scale

    def _read_register_short(self, register):
        buf = self.i2c.read_i2c_block_data(self.address, register, 2)
        return struct.unpack("<h", buf)[0]

    def _write_register_short(self, register, value):
        buf = struct.pack("<h", value)
        self.i2c.write_i2c_block_data(self.address, register, buf)

    def _read_register_three_shorts(self, register):
        buf = self.i2c.read_i2c_block_data(self.address, register, 6)
        return struct.unpack("<hhh", buf)

    def _read_register_char(self, register):
        return self.i2c.read_byte_data(self.address, register)

    def _write_register_char(self, register, value):
        self.i2c.write_byte_data(self.address, register, value)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
