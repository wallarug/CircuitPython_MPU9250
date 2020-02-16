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

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/wallarug/CircuitPython_MPU9250.git"

import time
try:
    import struct
except ImportError:
    import ustruct as struct

# not required
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_register.i2c_struct_array import StructArray

# required
import adafruit_bus_device.i2c_device as i2c_device
import adafruit_bus_device.spi_device as spi_device
from micropython import const


# Internal constants and register values:
# pylint: disable=bad-whitespace
_MPU9250_DEFAULT_ADDRESS    = const(0x69) # MPU9250 default i2c address
_MPU9250_DEVICE_ID          = const(0x71) # MPU9250 WHO_AM_I value

_MPU9250_INT_PIN_CFG        = const(0x37) # I2C Bypass enable configuration
_MPU9250_INT_ENABLE         = const(0x38) # Interrupt Enable
_MPU9250_INT_STATUS         = const(0x3A) # Interrupt Status

_MPU9250_I2C_MST_CTRL       = const(0x24) #
_MPU9250_WHO_AM_I           = const(0x75) # Device ID register

_MPU6500_DEFAULT_ADDRESS    = const(0x69) # MPU6500 default i2c address
_MPU6500_DEVICE_ID          = const(0x71) # MPU9250 WHO_AM_I value

_MPU6500_SELF_TEST_X      = const(0x0D) # Self test factory calibrated values register
_MPU6500_SELF_TEST_Y      = const(0x0E) # Self test factory calibrated values register
_MPU6500_SELF_TEST_Z      = const(0x0F) # Self test factory calibrated values register
_MPU6500_SELF_TEST_A      = const(0x10) # Self test factory calibrated values register
_MPU6500_SMPLRT_DIV       = const(0x19) # sample rate divisor register
_MPU6500_CONFIG           = const(0x1A) # General configuration register
_MPU6500_GYRO_CONFIG      = const(0x1B) # Gyro specfic configuration register
_MPU6500_ACCEL_CONFIG     = const(0x1C) # Accelerometer specific configration register
_MPU6500_ACCEL_CONFIG2    = const(0x1D) # Accelerometer config register
_MPU6500_INT_PIN_CONFIG   = const(0x37) # Interrupt pin configuration register
_MPU6500_INT_PIN_ENABLE   = const(0x38) # Interrupt pin enable register
_MPU6500_ACCEL_OUT        = const(0x3B) # base address for sensor data reads
_MPU6500_TEMP_OUT         = const(0x42) # Temperature data low byte register (low: 0x41)
_MPU6500_GYRO_OUT         = const(0x43) # base address for sensor data reads
_MPU6500_SIG_PATH_RESET   = const(0x68) # register to reset sensor signal paths
_MPU6500_PWR_MGMT_1       = const(0x6B) # Primary power/sleep control register
_MPU6500_PWR_MGMT_2       = const(0x6C) # Secondary power/sleep control register
_MPU6500_WHO_AM_I         = const(0x75) # Device ID register

_MPU6500_USER_CTRL          = const(0x6A) # FIFO and I2C Master control register
_MPU6500_I2C_SLV4_CTRL      = const(0x34) #
_MPU6500_I2C_MST_CTRL       = const(0x24) #
_MPU6500_I2C_SLV0_ADDR      = const(0x25) #
_MPU6500_I2C_SLV0_REG       = const(0x26) #
_MPU6500_I2C_SLV0_CTRL      = const(0x27) #
_MPU6500_I2C_SLV0_DO        = const(0x63) #
_MPU6500_I2C_MST_DELAY_CTRL = const(0x67) #
_MPU6500_EXT_SENS_DATA_00   = const(0x49) #


_AK8963_DEFAULT_ADDRESS    = const(0x0c) # AK8963 default i2c address
_AK8963_DEVICE_ID          = const(0x48) # MPU9250 WHO_AM_I value

_AK8963_WIA                = const(0x00) # Device ID register
_AK8963_INFO               = const(0x01) # Device Information register
_AK8963_ST1                = const(0x02) # Status register 1
_AK8963_MAG_OUT            = const(0x03) # base address for sensor data reads
_AK8963_HXL                = const(0x03) # 
_AK8963_HXH                = const(0x04)
_AK8963_HYL                = const(0x05)
_AK8963_HYH                = const(0x06)
_AK8963_HZL                = const(0x07)
_AK8963_HZH                = const(0x08)
_AK8963_ST2                = const(0x09)
_AK8963_CNTL1              = const(0x0A) # control register 1
_AK8963_CNTL2              = const(0x0B) # control register 2
_AK8963_ADJUST             = const(0x10) # base address for sensor adjust reads
_AK8963_ASAX               = const(0x10)
_AK8963_ASAY               = const(0x11)
_AK8963_ASAZ               = const(0x12)

_MAGTYPE                         = True
_XGTYPE                          = False

STANDARD_GRAVITY = 9.80665
# pylint: enable=bad-whitespace


def _twos_comp(val, bits):
    # Convert an unsigned integer in 2's compliment form of the specified bit
    # length to its signed integer value and return it.
    if val & (1 << (bits - 1)) != 0:
        return val - (1 << bits)
    return val

class MPU9250:
    """Driver for the MPU9250 9-DoF IMU accelerometer, magnetometer, gyroscope."""

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(6)

    def __init__(self):
        # defaults
        Ascale = AccelRange.RANGE_2_G
        Gscale = GyroRange.RANGE_500_DPS
        Mscale = MagSensitivity.16BIT
        Mmode = MagMode.MEASURE_100HZ
        sampleRate = 0x04

        self._filter_bandwidth = Bandwidth.BAND_260_HZ
        self._gyro_range = GyroRange.RANGE_500_DPS
        self._accel_range = AccelRange.RANGE_2_G
        
        # soft reset & reboot accel/gyro
        self._write_u8(_XGTYPE, _MPU6500_PWR_MGMT_1, 0x00)
        time.sleep(0.01)

        # Check ID registers.
        if self._read_u8(_XGTYPE, _MPU6500_WHO_AM_I) != _MPU6500_DEVICE_ID:# or \
           #self._read_u8(_MAGTYPE, _AK8963_WIA) != _AK8963_DEVICE_ID:
            raise RuntimeError('Could not find MPU9250, check wiring!')

        #self._write_u8(_XGTYPE, _MPU6500_SIG_PATH_RESET, 0x07)
        #time.sleep(0.01)

        # set stable timesource
        # Auto select clock source to be PLL gyroscope reference if ready else
        self._write_u8(_XGTYPE, _MPU6500_PWR_MGMT_1, 0x01)
        time.sleep(0.01)

        # Configure Gyro and Thermometer
        self._write_u8(_XGTYPE, _MPU6500_CONFIG, 0x03)

        # Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        self._write_u8(_XGTYPE, _MPU6500_SMPLRT_DIV, sampleRate)

        # Set Gyro full-scale range
        c = self._read_u8(_XGTYPE, _MPU6500_GYRO_CONFIG)
        c = c & ~0x02 # Clear Fchoice bits [1:0]
        c = c & ~0x18 # Clear AFS bits [4:3]
        c = c | Gscale << 3 # Set full scale range for the gyro
        self._write_u8(_XGTYPE, _MPU6500_GYRO_CONFIG, c)

        # Set accelerometer full-scale range configuration
        c = self._read_u8(_XGTYPE, _MPU6500_ACCEL_CONFIG)
        c = c & ~0x18 # Clear AFS bits [4:3]
        c = c | Ascale << 3 # Set full scale range for the accelerometer 
        self._write_u8(_XGTYPE, _MPU6500_ACCEL_CONFIG, c)

        # Set accelerometer sample rate configuration
        c = self._read_u8(_XGTYPE, _MPU6500_ACCEL_CONFIG2)
        c = c & ~0x0F # Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | 0x03 # Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        self._write_u8(_XGTYPE, _MPU6500_ACCEL_CONFIG2, c)


        # Magnetometer configuration values
        self._offset = (143.725, 6.00244, -21.6755)
        self._scale = (1.07464, 0.97619, 0.956875)
        self._adjustment = (0,0,0)

        # Configure Interrupts and Bypass Enable
        self.initAK8963()

    def read_temp_raw(self):
        """Read the raw temperature sensor value and return it as a 12-bit
        signed value.  If you want the temperature in nice units you probably
        want to use the temperature property!
        """
        # Read temp sensor - TODO: was low bit _MPU6500_TEMP_OUT
        self._read_bytes(_XGTYPE, 0x80 | _MPU6500_TEMP_OUT, 2,
                         self._BUFFER)
        temp = ((self._BUFFER[1] << 8) | self._BUFFER[0]) >> 4
        return _twos_comp(temp, 12)

    @property
    def temperature(self):
        """The current temperature in  º C"""
        raw_temperature = self.read_temp_raw()
        temp = (raw_temperature / 333.87) + 21.0
        return temp


    def read_accel_raw(self):
        """Read the raw accelerometer sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the acceleration in nice units you probably want to use the
        accelerometer property!
        """
        # Read the accelerometer
        self._read_bytes(_XGTYPE, 0x80 | _MPU6500_ACCEL_OUT, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('>hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)    

    @property
    def acceleration(self):
        """Acceleration X, Y, and Z axis data in m/s^2"""
        raw_data = self.read_accel_raw()
        raw_x = raw_data[0]
        raw_y = raw_data[1]
        raw_z = raw_data[2]

        accel_range = self._accel_range
        accel_scale = 1
        if accel_range == AccelRange.RANGE_16_G:
            accel_scale = 2048
        if accel_range == AccelRange.RANGE_8_G:
            accel_scale = 4096
        if accel_range == AccelRange.RANGE_4_G:
            accel_scale = 8192
        if accel_range == AccelRange.RANGE_2_G:
            accel_scale = 16384

        # setup range dependant scaling
        accel_x = (raw_x / accel_scale) * STANDARD_GRAVITY
        accel_y = (raw_y / accel_scale) * STANDARD_GRAVITY
        accel_z = (raw_z / accel_scale) * STANDARD_GRAVITY

        return (accel_x, accel_y, accel_z)

    def read_gyro_raw(self):
        """Read the raw gyroscope sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the gyroscope in nice units you probably want to use the
        gyroscope property!
        """
        # Read the gyroscope
        self._read_bytes(_XGTYPE, 0x80 | _MPU6500_GYRO_OUT, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('>hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def gyro(self):
        """Gyroscope X, Y, and Z axis data in º/s"""
        raw_data = self.read_gyro_raw()
        raw_x = raw_data[0]
        raw_y = raw_data[1]
        raw_z = raw_data[2]

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

    ## MAG
    def read_mag_raw(self):
        """Read the raw magnetometer sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the magnetometer in nice units you probably want to use the
        magnetometer property!
        """
        # Read the magnetometer
        self._read_bytes(_MAGTYPE, 0x80 | _AK8963_MAG_OUT, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)
    
    @property
    def magnetic(self):
        """The magnetometer X, Y, Z axis values as a 3-tuple of
        gauss values.
        """
        raw_data = self._raw_magnet_data
        #raw_x = _twos_comp(raw_data[0][0], 16)
        #raw_y = _twos_comp(raw_data[1][0], 16)
        #raw_z = _twos_comp(raw_data[2][0], 16)
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        print(raw_x, raw_y, raw_z)

        self._status # Enable updating readings again

        # Apply factory axial sensitivy adjustments
        #raw_x *= self._adjustment[0]
        #raw_y *= self._adjustment[1]
        #raw_z *= self._adjustment[2]

        # Apply output scale determined in constructor
        mag_range = self._mag_range
        mag_scale = 1
        if mag_range == Sensitivity.SENSE_16BIT:
            #mag_scale = 0.15 - for uT (micro-tesla)
            mag_scale = 1.499389499 # for mG (milliGauss) calc: 10.*4912./32760.0
        if mag_range == Sensitivity.SENSE_14BIT:
            #mag_scale = 0.6 - for uT (mico-tesla)
            mag_scale = 5.997557998 # for mG (millGauss) calc: 10.*4912./8190.0

        # setup range dependant scaling and offsets
        #mag_x = ((raw_x / mag_scale) - self._offset[0]) * self._scale[0]
        #mag_y = ((raw_y / mag_scale) - self._offset[1]) * self._scale[1]
        #mag_z = ((raw_z / mag_scale) - self._offset[2]) * self._scale[2]
        mag_x = (raw_x * mag_scale * self._scale[0]) - self._offset[0]
        mag_y = (raw_y * mag_scale * self._scale[1]) - self._offset[1]
        mag_z = (raw_z * mag_scale * self._scale[2]) - self._offset[1]

        return (mag_x, mag_y, mag_z)

    def read_gyro_calibration_raw(self):
        """Read the raw gyroscope calibration values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values. 
        """
        # Read the calibration
        self._read_bytes(_MAGTYPE, 0x80 | _AK8963_ASAX, 3,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<BBB', self._BUFFER[0:3])
        return (raw_x, raw_y, raw_z)

    def initAK8963(self, scale, mode):
        """ setup the AK8963 to be used with ONLY I2C native """
        # Enable I2C bypass to access for MPU9250 magnetometer access.
        self._write_u8(_XGTYPE, _MPU6500_INT_PIN_CONFIG, 0x12)
        self._write_u8(_XGTYPE, _MPU6500_INT_PIN_ENABLE, 0x01)
        time.sleep(0.1)

        # soft reset & reboot magnetometer
        self._write_u8(_MAGTYPE, _AK8963_CNTL, 0x00) # power down magnetometer
        time.sleep(0.01)
        self._write_u8(_MAGTYPE, _AK8963_CNTL, 0x0F) # enter fuse rom mode
        time.sleep(0.01)

        # factory calibration
        raw_adjustment = self.read_gyro_calibration_raw()
        asax = _twos_comp(raw_adjustment[0], 8)
        asay = _twos_comp(raw_adjustment[1], 8)
        asaz = _twos_comp(raw_adjustment[2], 8)

        print(asax, asay, asaz)

        self._adjustment = (
            ((asax - 128.0) / 256.0) + 1.0,
            ((asay - 128.0) / 256.0) + 1.0,
            ((asaz - 128.0) / 256.0) + 1.0
        )

        print(self._adjustment)

        # soft reset & reboot magnetometer
        self._write_u8(_MAGTYPE, _AK8963_CNTL, 0x00) # power down magnetometer
        time.sleep(0.01)
        self._write_u8(_MAGTYPE, _AK8963_CNTL, scale << 4 | mode) # Set magnetometer data resolution and sample ODR
        time.sleep(0.01)

    def initAK8963slave(self):
        """ setup the AK8963 to be used in slave mode """
        # Configure Interrupts and Bypass Enable
        self._write_u8(_XGTYPE, _MPU6500_INT_PIN_CONFIG, 0x10) # INT is 50 microsecond pulse and any read to clear 
        self._write_u8(_XGTYPE, _MPU6500_INT_PIN_ENABLE, 0x01) # Enable data ready (bit 0) interrupt
        time.sleep(0.01)

        self._write_u8(_XGTYPE, _MPU6500_USER_CTRL, 0x20) # Enable I2C Master mode
        self._write_u8(_XGTYPE, _MPU6500_I2C_MST_CTRL, 0x1D) # I2C configuration STOP after each transaction, master I2C bus at 400 KHz
        self._write_u8(_XGTYPE, _MPU6500_I2C_MST_DELAY_CTRL, 0x81) # Use blocking data retreival and enable delay for mag sample rate mismatch
        self._write_u8(_XGTYPE, _MPU6500_I2C_SLV4_CTRL, 0x01) # Delay mag data retrieval to once every other accel/gyro data sample


    def i2c_slave(self, addr, reg, data, ,size=1, read=False):

        if read:
            self._write_u8(_XGTYPE, _MPU6500_I2C_SLV0_ADDR, addr)
            self._write_u8(_XGTYPE, _MPU6500_I2C_SLV0_REG, reg)
            self._write_u8(_XGTYPE, _MPU6500_I2C_SLV0_DO, data)
            self._write_u8(_XGTYPE, _MPU6500_I2C_SLV0_CTRL, 0x80+size)
            time.sleep(0.05)
            
        else:
            self._write_u8(_XGTYPE, _MPU6500_I2C_SLV0_ADDR, addr | 0x80)
            self._write_u8(_XGTYPE, _MPU6500_I2C_SLV0_REG, reg)
            self._write_u8(_XGTYPE, _MPU6500_I2C_SLV0_CTRL, size)
            time.sleep(0.05)

            # TODO fix this:
            self._read_bytes(_XGTYPE, 0x80 | _MPU6500_EXT_SEN_DATA_00, size, self._BUFFER)
            raw_x, raw_y, raw_z = struct.unpack_from('<BBB', self._BUFFER[0:size])
            return (raw_x, raw_y, raw_z)
        
    

        

    def calibrate(self, count=256, delay=0.200):
        """
        Calibrate the magnetometer.

        The magnetometer needs to be turned in all possible directions
        during the callibration process. Ideally each axis would once 
        line up with the magnetic field.

        count: int
            Number of magnetometer readings that are taken for the calibration.
        
        delay: float
            Delay between the magntometer readings in seconds.
        """
        print("Starting Calibration.")
        print("The magnetometer needs to be turned in all possible directions \
        during the callibration process. Ideally each axis would once  \
        line up with the magnetic field.")
        
        self._offset = (0, 0, 0)
        self._scale = (1, 1, 1)

        raw_data = self._raw_magnet_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]
        self._status # Enable updating readings again
        
        minx = maxx = raw_x
        miny = maxy = raw_y
        minz = maxz = raw_z

        while count:
            sleep(delay)

            raw_data = self._raw_magnet_data
            print(raw_x, raw_y, raw_z)
            raw_x = raw_data[0][0]
            raw_y = raw_data[1][0]
            raw_z = raw_data[2][0]
            self._status # Enable updating readings again
            
            minx = min(minx, raw_x)
            maxx = max(maxx, raw_x)
            miny = min(miny, raw_y)
            maxy = max(maxy, raw_y)
            minz = min(minz, raw_z)
            maxz = max(maxz, raw_z)

            count -= 1

        # Hard iron correction
        offset_x = (maxx + minx) / 2
        offset_y = (maxy + miny) / 2
        offset_z = (maxz + minz) / 2

        self._offset = (offset_x, offset_y, offset_z)

        print("+++++++++++")
        print("Hard Iron Offset Values:")
        print(self._offset)

        # Soft iron correction
        avg_delta_x = (maxx - minx) / 2
        avg_delta_y = (maxy - miny) / 2
        avg_delta_z = (maxz - minz) / 2

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

        scale_x = avg_delta / avg_delta_x
        scale_y = avg_delta / avg_delta_y
        scale_z = avg_delta / avg_delta_z

        self._scale = (scale_x, scale_y, scale_z)

        print("Soft iron values")
        print(self._scale)
    
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
    def __init__(self, i2c, mag_address=_AK8963_DEFAULT_ADDRESS,
                 xg_address=_MPU6500_DEFAULT_ADDRESS):
        if xg_address in (0x68, 0x69): #and mag_address in (0x0c, 0x0b):
            #self._mag_device = i2c_device.I2CDevice(i2c, mag_address)
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



  

class AccelRange: # pylint: disable=too-few-public-methods
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
    
class MagSensitivity:
    """Allowed values for `range`.

    - ``Rate.CYCLE_1_25_HZ``
    - ``Rate.CYCLE_5_HZ``
    - ``Rate.CYCLE_20_HZ``
    - ``Rate.CYCLE_40_HZ``
    """
    14BIT = 0
    16BIT = 1

class MagMode:
    """Allowed values for `mode` setting

    - ``Mode.MODE_POWERDOWN``
    - ``Mode.MODE_SINGLE``
    - ``Mode.MODE_CONT1``
    - ``Mode.MODE_CONT2``
    - ``Mode.MODE_EXT_TRIG``
    - ``Mode.MODE_SELFTEST``
    - ``Mode.MODE_FUSE``

    """
    POWERDOWN = 0 #0b0000
    MEASURE_SINGLE = 1 #0b0001
    MEASURE_8HZ = 2 #0b0010 # 8 Hz (mode 1)
    EXT_TRIG = 4 #0b0100
    MEASURE_100HZ =  5 #0b0110 # 100 Hz (mode 2)
    SELFTEST = 8 #0b1000
    FUSE = 15 #0b1111        




