# First attempt at a library to communicate with the LSM9DS1
# with I2C protocol (without the external dependencies). 
# Based on the Adafruit_CircuitPython_LSM9DS1 library.
#
# Written by Madeline Corvin

# imports
# NOTE: The RaspberryPi will need to have the smbus2 module installed.
#       To check, use pip3 list. If smbus2 is not currently installed,
#       type pip3 install smbus2.
import time
try:
	import struct
except ImportError:
	import ustruct as struct
import smbus2
	

# Internal constants and register values:
# NOTE: may need to use const() for this
_LSM9DS1_ADDRESS_ACCELGYRO		= 0x6B
_LSM9DS1_ADDRESS_MAG			= 0x1E
_LSM9DS1_XG_ID				= 0b01101000
_LSM9DS1_MAG_ID				= 0b00111101
_LSM9DS1_ACCEL_MG_LSB_2G		= 0.061
_LSM9DS1_ACCEL_MG_LSB_4G		= 0.122
_LSM9DS1_ACCEL_MG_LSB_8G		= 0.244
_LSM9DS1_ACCEL_MG_LSB_16G		= 0.732
_LSM9DS1_MAG_MGAUSS_4GAUSS		= 0.14
_LSM9DS1_MAG_MGAUSS_8GAUSS		= 0.29
_LSM9DS1_MAG_MGAUSS_12GAUSS		= 0.43
_LSM9DS1_MAG_MGAUSS_16GAUSS		= 0.58
_LSM9DS1_GYRO_DPS_DIGIT_245DPS		= 0.00875
_LSM9DS1_GYRO_DPS_DIGIT_500DPS		= 0.01750
_LSM9DS1_GYRO_DPS_DIGIT_2000DPS 	= 0.07000
_LSM9DS1_TEMP_LSB_DEGREE_CELSIUS	= 8 # 1°C = 8, 25°C = 200, etc.
_LSM9DS1_REGISTER_WHO_AM_I_XG		= 0x0F
_LSM9DS1_REGISTER_CTRL_REG1_G		= 0x10
_LSM9DS1_REGISTER_CTRL_REG2_G		= 0x11
_LSM9DS1_REGISTER_CTRL_REG3_G		= 0x12
_LSM9DS1_REGISTER_TEMP_OUT_L		= 0x15
_LSM9DS1_REGISTER_TEMP_OUT_H		= 0x16
_LSM9DS1_REGISTER_STATUS_REG		= 0x17
_LSM9DS1_REGISTER_OUT_X_L_G		= 0x18
_LSM9DS1_REGISTER_OUT_X_H_G		= 0x19
_LSM9DS1_REGISTER_OUT_Y_L_G		= 0x1A
_LSM9DS1_REGISTER_OUT_Y_H_G		= 0x1B
_LSM9DS1_REGISTER_OUT_Z_L_G		= 0x1C
_LSM9DS1_REGISTER_OUT_Z_H_G		= 0x1D
_LSM9DS1_REGISTER_CTRL_REG4		= 0x1E
_LSM9DS1_REGISTER_CTRL_REG5_XL		= 0x1F
_LSM9DS1_REGISTER_CTRL_REG6_XL		= 0x20
_LSM9DS1_REGISTER_CTRL_REG7_XL		= 0x21
_LSM9DS1_REGISTER_CTRL_REG8		= 0x22
_LSM9DS1_REGISTER_CTRL_REG9		= 0x23
_LSM9DS1_REGISTER_CTRL_REG10		= 0x24
_LSM9DS1_REGISTER_OUT_X_L_XL		= 0x28
_LSM9DS1_REGISTER_OUT_X_H_XL		= 0x29
_LSM9DS1_REGISTER_OUT_Y_L_XL		= 0x2A
_LSM9DS1_REGISTER_OUT_Y_H_XL		= 0x2B
_LSM9DS1_REGISTER_OUT_Z_L_XL		= 0x2C
_LSM9DS1_REGISTER_OUT_Z_H_XL		= 0x2D
_LSM9DS1_REGISTER_WHO_AM_I_M		= 0x0F
_LSM9DS1_REGISTER_CTRL_REG1_M		= 0x20
_LSM9DS1_REGISTER_CTRL_REG2_M		= 0x21
_LSM9DS1_REGISTER_CTRL_REG3_M		= 0x22
_LSM9DS1_REGISTER_CTRL_REG4_M		= 0x23
_LSM9DS1_REGISTER_CTRL_REG5_M		= 0x24
_LSM9DS1_REGISTER_STATUS_REG_M		= 0x27
_LSM9DS1_REGISTER_OUT_X_L_M		= 0x28
_LSM9DS1_REGISTER_OUT_X_H_M		= 0x29
_LSM9DS1_REGISTER_OUT_Y_L_M		= 0x2A
_LSM9DS1_REGISTER_OUT_Y_H_M		= 0x2B
_LSM9DS1_REGISTER_OUT_Z_L_M		= 0x2C
_LSM9DS1_REGISTER_OUT_Z_H_M		= 0x2D
_LSM9DS1_REGISTER_CFG_M			= 0x30
_LSM9DS1_REGISTER_INT_SRC_M		= 0x31
_MAGTYPE				= True
_XGTYPE					= False
_SENSORS_GRAVITY_STANDARD		= 9.80665


# User facing constants/module globals
ACCELRANGE_2G		= (0b00 << 3)
ACCELRANGE_16G		= (0b01 << 3)
ACCELRANGE_4G		= (0b10 << 3)
ACCELRANGE_8G		= (0b11 << 3)
MAGGAIN_4GAUSS		= (0b00 << 5)  # +/- 4 gauss
MAGGAIN_8GAUSS		= (0b01 << 5)  # +/- 8 gauss
MAGGAIN_12GAUSS		= (0b10 << 5)  # +/- 12 gauss
MAGGAIN_16GAUSS		= (0b11 << 5)  # +/- 16 gauss
GYROSCALE_245DPS	= (0b00 << 3)  # +/- 245 degrees/s rotation
GYROSCALE_500DPS	= (0b01 << 3)  # +/- 500 degrees/s rotation
GYROSCALE_2000DPS	= (0b11 << 3)  # +/- 2000 degrees/s rotation


# def _twos_comp(val, bits):
	# Convert an unsigned integer in 2's complement form of the specified bit
	# length to its signed integer value and return it.
	#if val & (1 << (bits - 1)) != 0:
	#	return val - (1 << bits)
	#return val

class ContextManaged:
	# An object that automatically deinitializes hardware with a context manager.
	def __enter__(self):
		return self

	def __exit__(self, exc_type, exc_value, traceback):
		self.deinit()

	def deinit(self):
		# Free any hardware used by the object.
		return


class Lockable(ContextManaged):
	# An object that must be locked to prevent collisions on a microcontroller resource.
	_locked = False

	def try_lock(self):
		# Attempt to grab the lock. Return True on success, False if the lock is already taken.
		if self._locked:
			return False
		self._locked = True
		return True

	def unlock(self):
		# Release the lock so others may use the resource.
		if self._locked:
			self._locked = False
		else:
			raise ValueError("Not locked")


class I2CDevice(Lockable):
	MASTER = 0
	SLAVE = 1
	SDA = 3  # I2C SDA pin on the Raspberry Pi 3B
	SCL = 5  # I2C SCL pin on the Raspberry Pi 3B
	BUS = 1  # Using I2C bus 1 on the Pi

	def __init__(self, sensor_type, sda=SDA, scl=SCL, freq=400000, bus_num=BUS, mode=MASTER):
		self.deinit()
		if scl == SCL and sda == SDA:
			if mode != self.MASTER:
				raise NotImplementedError("Only I2C Master supported!")
			else:
				self._mode = self.MASTER

			try:
				self._i2c_bus = smbus2.SMBus(bus_num)
			except FileNotFoundError:
				raise RuntimeError("I2C Bus #%d not found!" % bus_num)

		else:
			raise NotImplementedError("No hardware I2C on ports!")
		
		# test bus connection
		if sensor_type == _MAGTYPE:
			device_address = _LSM9DS1_ADDRESS_MAG
		else:
			device_address = _LSM9DS1_ADDRESS_ACCELGYRO	
		self.try_bus(self,device_address)

		self._device_address = device_address

	def deinit(self):
		try:
			del self._mode
			del self._i2c_bus
			del self._device_address
		except AttributeError:
			pass

	def try_bus(self, device_address):
		while not self.try_lock():	# check lock
			pass
		try:
			# Try to read a byte from the device...
			# If you get an OSError, it means the device is not there
			self._i2c_bus.write_byte(device_address, b' ');
		except OSError:
			# Some devices don't like writing an empty bytesting...
			# Retry by reading a byte
			try:
				result = bytearray(1)
				result[0] = self._i2c_bus.read_byte(device_address)
			except OSError:
				raise ValueError("No I2C Device at address: %x" % device_address)
		finally:
			self.unlock()

	def read_from(self, reg_address):
		buf = bytearray(1)
		buf[0] = self._i2c_bus.read_byte_data(self._device_address, reg_address)
		return buf[0]
	
	def write(self, reg_address, value):
		self._i2c_bus.write_byte_data(self._device_address, reg_address, value)


class LSM9DS1_I2C(I2CDevice):
	# Driver for the LSM9DS1 accelerometer, gyroscope, and magnetometer,
	# connecting over I2C.	

	# Class level buffer for reading and writing data with the sensor.
	_BUFFER = bytearray(1)

	def __init__(self):
                # create attributes and set default ranges for sensors
                self._mag_device = I2CDevice(_MAGTYPE)
                self._xg_device = I2CDevice(_XGTYPE)
                self._accel_mg_lsb = None
                self._mag_mgauss_lsb = None
                self._gyro_dps_digit = None
                self.accel_range = ACCELRANGE_2G
                self.mag_gain = MAGGAIN_4GAUSS
                self.gyro_scale = GYROSCALE_245DPS
                # soft reset & reboot accel/gyro
                self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG8, 0x05)
                # soft reset & reboot magnetometer
                self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C)
                time.sleep(0.01)
                # Check ID Registers
                if self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_XG) != _LSM9DS1_XG_ID or \
                self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_M) != _LSM9DS1_MAG_ID:
                    raise RuntimeError("Could not find LSM9DS1, check wiring!")
                # enable gyro continuous
                self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0)
                # enable accelerometer continuous
                self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38)
                self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0)
                # enable mag continuous
                self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG3_M, 0x00)

	@property
	def accel_range(self):
                # The accelerometer range. Must be a value of:
                #	- ACCELRANGE_2G
                #	- ACCELRANGE_4G
                #	- ACCELRANGE_8G
                #	- ACCELRANGE_16G
                reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
                return (reg & 0b00011000) & 0xFF

	@accel_range.setter
	def accel_range(self, val):
                assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,ACCELRANGE_16G)
                reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
                reg = (reg & ~(0b00011000)) & 0xFF
                reg |= val
		self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, reg)
                if val == ACCELRANGE_2G:
                    self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_2G
                elif val == ACCELRANGE_4G:
                    self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_4G
                elif val == ACCELRANGE_8G:
                    self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_8G
                elif val == ACCELRANGE_16G:
                    self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_16G

	@property
	def mag_gain(self):
		# The magnetometer gain. Must be a value of:
		#	- MAGGAIN_4GAUSS
		#	- MAGGAIN_8GAUSS
		#	- MAGGAIN_12GAUSS
		#	- MAGGAIN_16GAUSS
		reg = self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
		return (reg & 0b01100000) & 0xFF

	@mag_gain.setter
	def mag_gain(self, val):
		assert val in (MAGGAIN_4GAUSS, MAGGAIN_8GAUSS, MAGGAIN_12GAUSS,
				MAGGAIN_16GAUSS)
		reg = self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
		reg = (reg & ~(0b01100000)) & 0xFF
		reg |= val
		self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, reg)
		if val == MAGGAIN_4GAUSS:
			self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_4GAUSS
		elif val == MAGGAIN_8GAUSS:
			self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_8GAUSS
		elif val == MAGGAIN_12GAUSS:
			self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_12GAUSS
		elif val == MAGGAIN_16GAUSS:
			self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_16GAUSS

	@property
	def gyro_scale(self):
		# The gyroscope scale. Must be a value of:
		#	- GYROSCALE_245DPS
		#	- GYROSCALE_500DPS
		#	- GYROSCALE_2000DPS
		reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
		return (reg & 0b00011000) & 0xFF
	
	@gyro_scale.setter
	def gyro_scale(self, val):
		assert val in (GYROSCALE_245DPS, GYROSCALE_500DPS, GYROSCALE_2000DPS)
		reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
		reg = (reg & ~(0b00011000)) & 0xFF
		reg |= val
		self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, reg)
		if val == GYROSCALE_245DPS:
			self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_245DPS
		elif val == GYROSCALE_500DPS:
			self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_500DPS
		elif val == GYROSCALE_2000DPS:
			self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_2000DPS
	
	def read_accel_raw(self):
		# Read the raw accelerometer sensor values and return it as
		# a 3-tuple of X, Y, Z axis values that are 16-bit unsigned values.
		# If you want the acceleration in nice units, you probably want to
		# use the accelerometer property!

		# read the accelerometer
		self._read_bytes(_XGTYPE, _LSM9DS1_REGISTER_OUT_X_L_XL, 6)
		raw_x, raw_y, raw_z = struct.unpackfrom('<hhh', self._BUFFER[0:6])
		return (raw_x, raw_y, raw_z)

	@property
	def acceleration(self):
		# The accelerometer X, Y, Z axis values as a 3-tuple of
		# m/s^2 values.
		raw = self.read_accel_raw()
		return map(lambda x: x * self._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD, raw)

	def read_mag_raw(self):
		# Read the raw magnetometer sensor values and return it as
		# a 3-tuple of X, Y, Z axis values that are 16-bit unsigned values.
		# If you want the magnetometer in nice units, you probably want to
		# use the magnetometer property!
	
		# read the magnetometer
		self._read_bytes(_MAGTYPE, _LSM9DS1_REGISTER_OUT_X_L_M, 6)
		raw_x, raw_y, raw_z = struct.unpackfrom('<hhh', self._BUFFER[0:6])
		return (raw_x, raw_y, raw_z)

	@property
	def magnetic(self):
		# The magnetometer X, Y, Z axis values as a 3-tuple of
		# gauss values.
		raw = self.read_mag_raw()
		return map(lambda x: x * self._mag_mgauss_lsb / 1000.0, raw)

	def read_gyro_raw(self):
		# Read the raw gyroscope sensro values and return it as
		# a 3-tuple of X, Y, Z axis values that are 16-bit unsigned values.
		# If you want the gyroscope in nice units, you probably want to 
		# use the gyroscope property!

		# Read the gyroscope
		self._read_bytes(_XGTYPE, _LSM9DS1_REGISTER_OUT_X_L_G, 6)
		raw_x, raw_y, raw_z = struct.unpackfrom('<hhh', self._BUFFER[0:6])
		return (raw_x, raw_y, raw_z)

	@property
	def gyro(self):
		# The gyroscope X, Y, Z axis as a 3-tuple of
		# degrees/second values.
		raw = self.read_gyro_raw()
		return map(lambda x: x * self._gyro_dps_digit, raw)

	#def read_temp_raw(self):
		# Read the raw temperature sensor value and return it as a
		# 12-bit signed value. If you want the temperature in nice units,
		# you probably want to use the temperature property!

		# read temp sensor
		#self._read_bytes(_XGTYPE, _LSM9DS1_REGISTER_TEMP_OUT_L, 2)
		#temp = ((self._BUFFER[1] << 8) | self._BUFFER[0]) >> 4	
		#return _twos_comp(temp, 12)

	#@property
	#def temperature(self):
		# The temperature of the sensor in degrees Celsius.
		#temp = self.read_temp_raw()
		#temp = 27.5 + temp/16
		#return temp

	def _read_u8(self, sensor_type, address):
		# Read an 8-bit unsigned value from the specified 8-bit address.
		# The sensor_type boolean should be _MAGTYPE when talking to the
		# magnetometer, and should be _XGTYPE when talking to the accel
		# or gyro.
		if sensor_type == _MAGTYPE:
			device = self._mag_device
		else:
			device = self._xg_device
		with device as i2c:
			address &= 0xFF
			self._BUFFER[0] = i2c.read_from(address)
		return self._BUFFER[0]

	def _read_bytes(self, sensor_type, address, count):
		# Read a count number of bytes into _BUFFER starting from the
		# provided address. The sensor_type boolean should be _MAGTYPE
		# when talking to the magnetometer and should be _XGTYPE when
		# talking to the accel or gyro.
		current_addr = address
		if sensor_type == _MAGTYPE:
			device = self._mag_device
		else:
			device = self._xg_device
		with device as i2c:
			current_addr &= 0xFF
			for x in range(0,count):
				self._BUFFER[x] = read_from(current_addr)
				current_addr += 0x01

	def _write_u8(self, sensor_type, address, value):
		# Write an 8-bit unsigned value to the specified 8-bit address.
		# The sensor_type boolean should be _MAGTYPE when talking to the
		# magnetometer, and should be _XGTYPE when talking to the accel or
		# gyro.
		if sensor_type == _MAGTYPE:
			device = self._mag_device
		else:
			device = self._xg_device
		with device as i2c:
			address &= 0xFF
			i2c.write(address, value)	
