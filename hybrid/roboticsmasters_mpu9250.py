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
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_struct import ROUnaryStruct

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
_MPU9250_DEFAULT_ADDRESS    = 0x69 # MPU9250 default i2c address
_MPU9250_DEVICE_ID          = 0x71 # MPU9250 WHO_AM_I value

_MPU6500_DEFAULT_ADDRESS    = 0x69 # MPU6500 default i2c address
_AK8963_DEFAULT_ADDRESS     = 0x0c # AK8963 default i2c address


_MPU9250_INT_PIN_CFG        = 0x37 # I2C Bypass enable configuration
_MPU9250_INT_ENABLE         = 0x38 # Interrupt Enable
_MPU9250_INT_STATUS         = 0x3A # Interrupt Status

_MPU9250_I2C_MST_CTRL       = 0x24 #
_MPU9250_WHO_AM_I           = 0x75 # Device ID register


# pylint: enable=bad-whitespace
_MPU6500_DEFAULT_ADDRESS    = 0x69 # MPU6500 default i2c address
_MPU6500_DEVICE_ID          = 0x71 # MPU9250 WHO_AM_I value

_MPU6500_SELF_TEST_X      = 0x0D # Self test factory calibrated values register
_MPU6500_SELF_TEST_Y      = 0x0E # Self test factory calibrated values register
_MPU6500_SELF_TEST_Z      = 0x0F # Self test factory calibrated values register
_MPU6500_SELF_TEST_A      = 0x10 # Self test factory calibrated values register
_MPU6500_SMPLRT_DIV       = 0x19 # sample rate divisor register
_MPU6500_CONFIG           = 0x1A # General configuration register
_MPU6500_GYRO_CONFIG      = 0x1B # Gyro specfic configuration register
_MPU6500_ACCEL_CONFIG     = 0x1C # Accelerometer specific configration register
_MPU6500_INT_PIN_CONFIG   = 0x37 # Interrupt pin configuration register
_MPU6500_ACCEL_OUT        = 0x3B # base address for sensor data reads
_MPU6500_TEMP_OUT         = 0x41 # Temperature data high byte register
_MPU6500_GYRO_OUT         = 0x43 # base address for sensor data reads
_MPU6500_SIG_PATH_RESET   = 0x68 # register to reset sensor signal paths
_MPU6500_USER_CTRL        = 0x6A # FIFO and I2C Master control register
_MPU6500_PWR_MGMT_1       = 0x6B # Primary power/sleep control register
_MPU6500_PWR_MGMT_2       = 0x6C # Secondary power/sleep control register
_MPU6500_WHO_AM_I         = 0x75 # Device ID register





class MPU9250:
    """Driver for the MPU9250 9-DoF IMU.
        :param ~busio.I2C i2c_bus: The I2C bus the MPU9250 is connected to.
        :param address: The I2C slave address of the sensor
    """

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(6)

    def __init__(self, i2c_bus,
                 mpu_addr=_MPU6500_DEFAULT_ADDRESS,
                 akm_addr=_AK8963_DEFAULT_ADDRESS):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, mpu_addr)

        if self._device_id != _MPU9250_DEVICE_ID:
            raise RuntimeError("Failed to find MPU9250 - check your wiring!")

        self._mpu = MPU6500(i2c_bus, mpu_addr)

        self._bypass = 1
        self._ready = 1
        sleep(0.100)
        
        self._akm = AK8963(i2c_bus, akm_addr)

    @property
    def temperature(self):
        """The current temperature in  º C"""
        raw_temperature = self._raw_temp_data
        #temp = (raw_temperature + 12412.0) / 340.0
        #temp = (raw_temperature / 340.0) + 36.53
        temp = (raw_temperature / 333.87) + 21.0
        return temp

    @property
    def acceleration(self):
        """Acceleration X, Y, and Z axis data in m/s^2"""
        raw_data = self._raw_accel_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        accel_range = self._accel_range
        accel_scale = 1
        if accel_range == Range.RANGE_16_G:
            accel_scale = 2048
        if accel_range == Range.RANGE_8_G:
            accel_scale = 4096
        if accel_range == Range.RANGE_4_G:
            accel_scale = 8192
        if accel_range == Range.RANGE_2_G:
            accel_scale = 16384

        # setup range dependant scaling
        accel_x = (raw_x / accel_scale) * STANDARD_GRAVITY
        accel_y = (raw_y / accel_scale) * STANDARD_GRAVITY
        accel_z = (raw_z / accel_scale) * STANDARD_GRAVITY

        return (accel_x, accel_y, accel_z)

    @property
    def gyro(self):
        """Gyroscope X, Y, and Z axis data in º/s"""
        raw_data = self._raw_gyro_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        gyro_scale = 1
        gyro_range = self._gyro_range
        if gyro_range == GyroRange.RANGE_250_DPS:
            gyro_scale = 131
        if gyro_range == GyroRange.RANGE_500_DPS:
            gyro_scale = 62.5
        if gyro_range == GyroRange.RANGE_1000_DPS:
            gyro_scale = 32.8
        if gyro_range == GyroRange.RANGE_2000_DPS:
            gyro_scale = 16.4

        # setup range dependant scaling
        gyro_x = (raw_x / gyro_scale)
        gyro_y = (raw_y / gyro_scale)
        gyro_z = (raw_z / gyro_scale)

        return (gyro_x, gyro_y, gyro_z)

    @property
    def cycle(self):
        """Enable or disable perodic measurement at a rate set by `cycle_rate`.
        If the sensor was in sleep mode, it will be waken up to cycle"""
        return self._cycle

    @cycle.setter
    def cycle(self, value):
        self.sleep = not value
        self._cycle = value

    @property
    def gyro_range(self):
        """The measurement range of all gyroscope axes. Must be a `GyroRange`"""
        return self._gyro_range

    @gyro_range.setter
    def gyro_range(self, value):
        if (value < 0) or (value > 3):
            raise ValueError("gyro_range must be a GyroRange")
        self._gyro_range = value
        sleep(0.01)

    @property
    def accelerometer_range(self):
        """The measurement range of all accelerometer axes. Must be a `Range`"""
        return self._accel_range

    @accelerometer_range.setter
    def accelerometer_range(self, value):
        if (value < 0) or (value > 3):
            raise ValueError("accelerometer_range must be a Range")
        self._accel_range = value
        sleep(0.01)

    @property
    def filter_bandwidth(self):
        """The bandwidth of the gyroscope Digital Low Pass Filter. Must be a `GyroRange`"""
        return self._filter_bandwidth

    @filter_bandwidth.setter
    def filter_bandwidth(self, value):
        if (value < 0) or (value > 6):
            raise ValueError("filter_bandwidth must be a Bandwidth")
        self._filter_bandwidth = value
        sleep(0.01)

    @property
    def cycle_rate(self):
        """The rate that measurements are taken while in `cycle` mode. Must be a `Rate`"""
        return self._cycle_rate

    @cycle_rate.setter
    def cycle_rate(self, value):
        if (value < 0) or (value > 3):
            raise ValueError("cycle_rate must be a Rate")
        self._cycle_rate = value
        sleep(0.01)

    
    ## DEFAULT FROM LSM DRIVER

    def _read_u8(self, sensor_type, address):
        # Read an 8-bit unsigned value from the specified 8-bit address.
        # The sensor_type boolean should be _MAGTYPE when talking to the
        # magnetometer, or _XGTYPE when talking to the accel or gyro.
        # MUST be implemented by subclasses!
        raise NotImplementedError()

    def _read_bytes(self, sensor_type, address, count, buf):
        # Read a count number of bytes into buffer from the provided 8-bit
        # register address.  The sensor_type boolean should be _MAGTYPE when
        # talking to the magnetometer, or _XGTYPE when talking to the accel or
        # gyro.  MUST be implemented by subclasses!
        raise NotImplementedError()

    def _write_u8(self, sensor_type, address, val):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        # The sensor_type boolean should be _MAGTYPE when talking to the
        # magnetometer, or _XGTYPE when talking to the accel or gyro.
        # MUST be implemented by subclasses!
        raise NotImplementedError()

    def reset(self):
        """Reinitialize the sensor"""
        self._mpu.reset()
        self._akm.reset()

    _device_id = ROUnaryStruct(_MPU9250_WHO_AM_I, ">B")
    _bypass = RWBit(_MPU9250_INT_PIN_CFG, 1, 1)
    _ready = RWBit(_MPU9250_INT_ENABLE, 0, 1)

    @property
    def temperature(self):
        """The current temperature in  º C"""
        return self._mpu.temperature

    @property
    def acceleration(self):
        """Acceleration X, Y, and Z axis data in m/s^2"""
        return self._mpu.acceleration

    @property
    def gyro(self):
        """Gyroscope X, Y, and Z axis data in º/s"""
        return self._mpu.gyro

    @property
    def magnetic(self):
        """Magnetometer X, Y and Z asix data in micro-Tesla (uT)"""
        return self._akm.magnetic

    def cal_mag(self):
        return self._akm.calibrate()


class MPU9250_I2C(MPU9250):
    """Driver for the MPU9250 connect over I2C.
    :param ~busio.I2C i2c: The I2C bus object used to connect to the MPU9250.
        .. note:: This object should be shared among other driver classes that use the
            same I2C bus (SDA & SCL pins) to connect to different I2C devices.
    :param int mag_address: A 8-bit integer that represents the i2c address of the
        MPU9250's magnetometer. Options are limited to ``0x0c``.
        Defaults to ``0x0c``.
    :param int xg_address: A 8-bit integer that represents the i2c address of the
        MPU9250's accelerometer and gyroscope. Options are limited to ``0x40`` or ``0x41``.
        Defaults to ``0x41``.
    """
    def __init__(self, i2c, mag_address=_LSM9DS1_ADDRESS_MAG,
                 xg_address=_LSM9DS1_ADDRESS_ACCELGYRO):
        if mag_address in (0x1c, 0x1e) and xg_address in (0x6a, 0x6b):
            self._mag_device = i2c_device.I2CDevice(i2c, mag_address)
            self._xg_device = i2c_device.I2CDevice(i2c, xg_address)
            super().__init__()
        else:
            raise ValueError('address parmeters are incorrect. Read the docs at '
                             'circuitpython.rtfd.io/projects/lsm9ds1/en/latest'
                             '/api.html#adafruit_lsm9ds1.LSM9DS1_I2C')

    def _read_u8(self, sensor_type, address):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_start=1, in_end=2)
        return self._BUFFER[1]

    def _read_bytes(self, sensor_type, address, count, buf):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            buf[0] = address & 0xFF
            i2c.write_then_readinto(buf, buf, out_end=1, in_end=count)

    def _write_u8(self, sensor_type, address, val):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)


class MPU9250_SPI(MPU9250):
    """Driver for the MPU9250 connect over SPI.
    :param ~busio.SPI spi: The SPI bus object used to connect to the MPU9250.
        .. note:: This object should be shared among other driver classes that use the
            same SPI bus (SCK, MISO, MOSI pins) to connect to different SPI devices.
    :param ~digitalio.DigitalInOut mcs: The digital output pin connected to the
        LSM9DS1's CSM (Chip Select Magnetometer) pin.
    :param ~digitalio.DigitalInOut xgcs: The digital output pin connected to the
        LSM9DS1's CSAG (Chip Select Accelerometer/Gyroscope) pin.
    """
    # pylint: disable=no-member
    def __init__(self, spi, xgcs, mcs):
        self._mag_device = spi_device.SPIDevice(spi, mcs, baudrate=200000, phase=1, polarity=1)
        self._xg_device = spi_device.SPIDevice(spi, xgcs, baudrate=200000, phase=1, polarity=1)
        super().__init__()

    def _read_u8(self, sensor_type, address):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as spi:
            self._BUFFER[0] = (address | 0x80) & 0xFF
            spi.write(self._BUFFER, end=1)
            spi.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def _read_bytes(self, sensor_type, address, count, buf):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as spi:
            buf[0] = (address | 0x80) & 0xFF
            spi.write(buf, end=1)
            spi.readinto(buf, end=count)

    def _write_u8(self, sensor_type, address, val):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as spi:
            self._BUFFER[0] = (address & 0x7F) & 0xFF
            self._BUFFER[1] = val & 0xFF
            spi.write(self._BUFFER, end=2)



  

class Range: # pylint: disable=too-few-public-methods
    """Allowed values for `accelerometer_range`.
    - ``Range.RANGE_2_G``
    - ``Range.RANGE_4_G``
    - ``Range.RANGE_8_G``
    - ``Range.RANGE_16_G``
    """
    RANGE_2_G = 0  # +/- 2g (default value)
    RANGE_4_G = 1  # +/- 4g
    RANGE_8_G = 2  # +/- 8g
    RANGE_16_G = 3 # +/- 16g

class GyroRange: # pylint: disable=too-few-public-methods
    """Allowed values for `gyro_range`.
    - ``GyroRange.RANGE_250_DPS``
    - ``GyroRange.RANGE_500_DPS``
    - ``GyroRange.RANGE_1000_DPS``
    - ``GyroRange.RANGE_2000_DPS``
    """
    RANGE_250_DPS = 0  # +/- 250 deg/s (default value)
    RANGE_500_DPS = 1  # +/- 500 deg/s
    RANGE_1000_DPS = 2 # +/- 1000 deg/s
    RANGE_2000_DPS = 3 # +/- 2000 deg/s

class Bandwidth: # pylint: disable=too-few-public-methods
    """Allowed values for `filter_bandwidth`.
    - ``Bandwidth.BAND_260_HZ``
    - ``Bandwidth.BAND_184_HZ``
    - ``Bandwidth.BAND_94_HZ``
    - ``Bandwidth.BAND_44_HZ``
    - ``Bandwidth.BAND_21_HZ``
    - ``Bandwidth.BAND_10_HZ``
    - ``Bandwidth.BAND_5_HZ``
    """
    BAND_260_HZ = 0 # Docs imply this disables the filter
    BAND_184_HZ = 1 # 184 Hz
    BAND_94_HZ = 2  # 94 Hz
    BAND_44_HZ = 3  # 44 Hz
    BAND_21_HZ = 4  # 21 Hz
    BAND_10_HZ = 5  # 10 Hz
    BAND_5_HZ = 6   # 5 Hz

class Rate: # pylint: disable=too-few-public-methods
    """Allowed values for `cycle_rate`.
    - ``Rate.CYCLE_1_25_HZ``
    - ``Rate.CYCLE_5_HZ``
    - ``Rate.CYCLE_20_HZ``
    - ``Rate.CYCLE_40_HZ``
    """
    CYCLE_1_25_HZ = 0 # 1.25 Hz
    CYCLE_5_HZ = 1    # 5 Hz
    CYCLE_20_HZ = 2   # 20 Hz
    CYCLE_40_HZ = 3   # 40 Hz    
    
        



