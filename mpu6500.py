# Copyright (c) 2018-2019 Mika Tuupola
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

# https://github.com/tuupola/micropython-mpu9250

"""
MicroPython I2C driver for MPU6500 6-axis motion tracking device
"""

__version__ = "0.1.0-a"

# pylint: disable=import-error
import ustruct
import utime
#from machine import I2C, Pin
import Adafruit_GPIO.I2C
#from micropython import const
# pylint: enable=import-error

_GYRO_CONFIG = 0x1b
_ACCEL_CONFIG = 0x1c
_ACCEL_CONFIG2 = 0x1d
_INT_PIN_CFG = 0x37
_ACCEL_XOUT_H = 0x3b
_ACCEL_XOUT_L = 0x3c
_ACCEL_YOUT_H = 0x3d
_ACCEL_YOUT_L = 0x3e
_ACCEL_ZOUT_H = 0x3f
_ACCEL_ZOUT_L= 0x40
_TEMP_OUT_H = 0x41
_TEMP_OUT_L = 0x42
_GYRO_XOUT_H = 0x43
_GYRO_XOUT_L = 0x44
_GYRO_YOUT_H = 0x45
_GYRO_YOUT_L = 0x46
_GYRO_ZOUT_H = 0x47
_GYRO_ZOUT_L = 0x48
_WHO_AM_I = 0x75

#_ACCEL_FS_MASK = 0b00011000
ACCEL_FS_SEL_2G = 0b00000000
ACCEL_FS_SEL_4G = 0b00001000
ACCEL_FS_SEL_8G = 0b00010000
ACCEL_FS_SEL_16G = 0b00011000

_ACCEL_SO_2G = 16384 # 1 / 16384 ie. 0.061 mg / digit
_ACCEL_SO_4G = 8192 # 1 / 8192 ie. 0.122 mg / digit
_ACCEL_SO_8G = 4096 # 1 / 4096 ie. 0.244 mg / digit
_ACCEL_SO_16G = 2048 # 1 / 2048 ie. 0.488 mg / digit

#_GYRO_FS_MASK = 0b00011000
GYRO_FS_SEL_250DPS = 0b00000000
GYRO_FS_SEL_500DPS = 0b00001000
GYRO_FS_SEL_1000DPS = 0b00010000
GYRO_FS_SEL_2000DPS = 0b00011000

_GYRO_SO_250DPS = 131
_GYRO_SO_500DPS = 62.5
_GYRO_SO_1000DPS = 32.8
_GYRO_SO_2000DPS = 16.4

_TEMP_SO = 333.87
_TEMP_OFFSET = 21

# Used for enablind and disabling the i2c bypass access
_I2C_BYPASS_MASK = 0b00000010
_I2C_BYPASS_EN = 0b00000010
_I2C_BYPASS_DIS = 0b00000000

SF_G = 1
SF_M_S2 = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 0.017453292519943 # 1 deg/s is 0.017453292519943 rad/s

class MPU6500:
    """Class which provides interface to MPU6500 6-axis motion tracking device."""
    def __init__(
        self, i2c_interface=None, busnum=1, address=0x68,
        accel_fs=ACCEL_FS_SEL_2G, gyro_fs=GYRO_FS_SEL_250DPS,
        accel_sf=SF_M_S2, gyro_sf=SF_RAD_S,
        gyro_offset=(0, 0, 0)
    ):
        if i2c_interface is None:
            # Use pure python I2C interface if none is specified.
            import Adafruit_PureIO.smbus
            self.i2c = Adafruit_PureIO.smbus.SMBus(busnum)
        else:
            # Otherwise use the provided class to create an smbus interface.
            self.i2c = i2c_interface(busnum)

        #self.i2c = i2c
        self.address = address

        if 0x71 != self.whoami:
            raise RuntimeError("MPU6500 not found in I2C bus.")

        self._accel_so = self._accel_fs(accel_fs)
        self._gyro_so = self._gyro_fs(gyro_fs)
        self._accel_sf = accel_sf
        self._gyro_sf = gyro_sf
        self._gyro_offset = gyro_offset

        # Enable I2C bypass to access for MPU9250 magnetometer access.
        char = self._read_register_char(_INT_PIN_CFG)
        char &= ~_I2C_BYPASS_MASK # clear I2C bits
        char |= _I2C_BYPASS_EN
        self._write_register_char(_INT_PIN_CFG, char)

    @property
    def acceleration(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. Will
        return values in g if constructor was provided `accel_sf=SF_M_S2`
        parameter.
        """
        so = self._accel_so
        sf = self._accel_sf

        xyz = self._read_register_three_shorts(_ACCEL_XOUT_H)
        return tuple([value / so * sf for value in xyz])

    @property
    def gyro(self):
        """
        X, Y, Z radians per second as floats.
        """
        so = self._gyro_so
        sf = self._gyro_sf
        ox, oy, oz = self._gyro_offset

        xyz = self._read_register_three_shorts(_GYRO_XOUT_H)
        xyz = [value / so * sf for value in xyz]

        xyz[0] -= ox
        xyz[1] -= oy
        xyz[2] -= oz

        return tuple(xyz)

    @property
    def temperature(self):
        """
        Die temperature in celcius as a float.
        """
        temp = self._read_register_short(_TEMP_OUT_H)
        return ((temp - _TEMP_OFFSET) / _TEMP_SO) + _TEMP_OFFSET

    @property
    def whoami(self):
        """ Value of the whoami register. """
        return self._read_register_char(_WHO_AM_I)

    def calibrate_gyro(self, count=256, delay=0):
        ox, oy, oz = (0.0, 0.0, 0.0)
        self._gyro_offset = (0.0, 0.0, 0.0)
        n = float(count)

        while count:
            utime.sleep_ms(delay)
            gx, gy, gz = self.gyro
            ox += gx
            oy += gy
            oz += gz
            count -= 1

        self._gyro_offset = (ox / n, oy / n, oz / n)
        return self._gyro_offset

    def _read_register_short(self, register, buf=bytearray(2)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack(">h", buf)[0]

    def _write_register_short(self, register, value, buf=bytearray(2)):
        ustruct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _read_register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack(">hhh", buf)

    def _read_register_char(self, register, buf=bytearray(1)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return buf[0]

    def _write_register_char(self, register, value, buf=bytearray(1)):
        ustruct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _accel_fs(self, value):
        self._write_register_char(_ACCEL_CONFIG, value)

        # Return the sensitivity divider
        if ACCEL_FS_SEL_2G == value:
            return _ACCEL_SO_2G
        elif ACCEL_FS_SEL_4G == value:
            return _ACCEL_SO_4G
        elif ACCEL_FS_SEL_8G == value:
            return _ACCEL_SO_8G
        elif ACCEL_FS_SEL_16G == value:
            return _ACCEL_SO_16G

    def _gyro_fs(self, value):
        self._write_register_char(_GYRO_CONFIG, value)

        # Return the sensitivity divider
        if GYRO_FS_SEL_250DPS == value:
            return _GYRO_SO_250DPS
        elif GYRO_FS_SEL_500DPS == value:
            return _GYRO_SO_500DPS
        elif GYRO_FS_SEL_1000DPS == value:
            return _GYRO_SO_1000DPS
        elif GYRO_FS_SEL_2000DPS == value:
            return _GYRO_SO_2000DPS

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
