''' RoombaCI_lib.py
Purpose: Python Library with LSM9DS1 class and specific functions
	and iRobot Create 2 (Roomba) class and specific functions
Import this file in main Python file to access functions
Made by: Timothy Anglea, Joshua Harvey, Madeline Corvin
Last Modified: 06/05/2019
'''
## Import Libraries ##
from ctypes import * # May not need this anymore
import time
import math
import serial
import numpy as np
try: # May not need this either (?)
	import struct
except ImportError:
	import ustruct as struct
# NOTE: The RaspberryPi will need to have the smbus2 module installed.
#		To check, use pip3 list. If smbus2 is not currently installed,
#		type 'pip3 install smbus2'.
import smbus2

## Important constants and functions for LSM9DS1 IMU ##
# Internal constants and register values:
# NOTE: may need to use const() for this
_LSM9DS1_ADDRESS_ACCELGYRO			= 0x6B
_LSM9DS1_ADDRESS_MAG				= 0x1E
_LSM9DS1_XG_ID						= 0b01101000
_LSM9DS1_MAG_ID						= 0b00111101
_LSM9DS1_ACCEL_MG_LSB_2G			= 0.061
_LSM9DS1_ACCEL_MG_LSB_4G			= 0.122
_LSM9DS1_ACCEL_MG_LSB_8G			= 0.244
_LSM9DS1_ACCEL_MG_LSB_16G			= 0.732
_LSM9DS1_MAG_MGAUSS_4GAUSS			= 0.14
_LSM9DS1_MAG_MGAUSS_8GAUSS			= 0.29
_LSM9DS1_MAG_MGAUSS_12GAUSS			= 0.43
_LSM9DS1_MAG_MGAUSS_16GAUSS			= 0.58
_LSM9DS1_GYRO_DPS_DIGIT_245DPS		= 0.00875
_LSM9DS1_GYRO_DPS_DIGIT_500DPS		= 0.01750
_LSM9DS1_GYRO_DPS_DIGIT_2000DPS		= 0.07000
_LSM9DS1_TEMP_LSB_DEGREE_CELSIUS	= 8 # 1°C = 8, 25°C = 200, etc.
_LSM9DS1_REGISTER_WHO_AM_I_XG		= 0x0F
_LSM9DS1_REGISTER_CTRL_REG1_G		= 0x10
_LSM9DS1_REGISTER_CTRL_REG2_G		= 0x11
_LSM9DS1_REGISTER_CTRL_REG3_G		= 0x12
_LSM9DS1_REGISTER_TEMP_OUT_L		= 0x15
_LSM9DS1_REGISTER_TEMP_OUT_H		= 0x16
_LSM9DS1_REGISTER_STATUS_REG		= 0x17
_LSM9DS1_REGISTER_OUT_X_L_G			= 0x18
_LSM9DS1_REGISTER_OUT_X_H_G			= 0x19
_LSM9DS1_REGISTER_OUT_Y_L_G			= 0x1A
_LSM9DS1_REGISTER_OUT_Y_H_G			= 0x1B
_LSM9DS1_REGISTER_OUT_Z_L_G			= 0x1C
_LSM9DS1_REGISTER_OUT_Z_H_G			= 0x1D
_LSM9DS1_REGISTER_CTRL_REG4			= 0x1E
_LSM9DS1_REGISTER_CTRL_REG5_XL		= 0x1F
_LSM9DS1_REGISTER_CTRL_REG6_XL		= 0x20
_LSM9DS1_REGISTER_CTRL_REG7_XL		= 0x21
_LSM9DS1_REGISTER_CTRL_REG8			= 0x22
_LSM9DS1_REGISTER_CTRL_REG9			= 0x23
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
_LSM9DS1_REGISTER_OUT_X_L_M			= 0x28
_LSM9DS1_REGISTER_OUT_X_H_M			= 0x29
_LSM9DS1_REGISTER_OUT_Y_L_M			= 0x2A
_LSM9DS1_REGISTER_OUT_Y_H_M			= 0x2B
_LSM9DS1_REGISTER_OUT_Z_L_M			= 0x2C
_LSM9DS1_REGISTER_OUT_Z_H_M			= 0x2D
_LSM9DS1_REGISTER_CFG_M				= 0x30
_LSM9DS1_REGISTER_INT_SRC_M			= 0x31
_MAGTYPE							= True
_XGTYPE								= False
_SENSORS_GRAVITY_STANDARD			= 9.80665


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

def _twos_comp(val, bits):
	# Convert an unsigned integer, val, of the specified bit length, bits,
	# to its signed integer value in 2's complement form and return it.
	if val & (1 << (bits - 1)) != 0:
		return val - (1 << bits)
	return val

class ContextManaged:
	# An object that automatically deinitializes hardware with a context manager.
	def __enter__(self):
		return self

	def __exit__(self, exc_type, exc_value, traceback):
		#self.deinit()
		pass

	def deinit(self):
		# Free any hardware used by the object.
		return

class I2CDevice(ContextManaged):
	MASTER = 0
	SLAVE = 1
	SDA = 3  # I2C SDA pin on the Raspberry Pi 3B
	SCL = 5  # I2C SCL pin on the Raspberry Pi 3B
	BUS = 1  # Using I2C bus 1 on the Pi

	def __init__(self, sensor_type, sda=SDA, scl=SCL, freq=400000, bus_num=BUS, mode=MASTER):
		if scl == self.SCL and sda == self.SDA:
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
		
		# set device address
		if sensor_type == _MAGTYPE:
			device_address = _LSM9DS1_ADDRESS_MAG
		else:
			device_address = _LSM9DS1_ADDRESS_ACCELGYRO
		
		self._device_address = device_address
		
	def read_from(self, reg_address):
		buf = bytearray(1)
		return self._i2c_bus.read_byte_data(self._device_address, reg_address)
	
	def write(self, reg_address, value):
		self._i2c_bus.write_byte_data(self._device_address, reg_address, value)

##################################################################
## LSM9DS1 IMU Class ##
class LSM9DS1_I2C(I2CDevice):
	# Driver for the LSM9DS1 accelerometer, gyroscope, and magnetometer,
	# connecting over I2C.  
	
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
		# Calibration offset variables
		self.m_offset = [0.0, 0.0, 0.0]

	@property
	def accel_range(self):
		# The accelerometer range. Must be a value of:
		#   - ACCELRANGE_2G
		#   - ACCELRANGE_4G
		#   - ACCELRANGE_8G
		#   - ACCELRANGE_16G
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
		#   - MAGGAIN_4GAUSS
		#   - MAGGAIN_8GAUSS
		#   - MAGGAIN_12GAUSS
		#   - MAGGAIN_16GAUSS
		reg = self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
		return (reg & 0b01100000) & 0xFF
	
	@mag_gain.setter
	def mag_gain(self, val):
		assert val in (MAGGAIN_4GAUSS, MAGGAIN_8GAUSS, MAGGAIN_12GAUSS, MAGGAIN_16GAUSS)
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
		#   - GYROSCALE_245DPS
		#   - GYROSCALE_500DPS
		#   - GYROSCALE_2000DPS
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
		# Read the raw accelerometer sensor values and returns it as
		# a list of X, Y, Z axis values that are 16-bit unsigned values.
		# If you want the acceleration in nice units, you probably want to
		# use the accelerometer property!
		
		# read the accelerometer
		buf = self._read_bytes(_XGTYPE, _LSM9DS1_REGISTER_OUT_X_L_XL, 6)
		raw_x = (buf[1] << 8) + buf[0]
		raw_x = _twos_comp(raw_x, 16)
		raw_y = (buf[3] << 8) + buf[2]
		raw_y = _twos_comp(raw_y, 16)
		raw_z = (buf[5] << 8) + buf[4]
		raw_z = _twos_comp(raw_z, 16)
		return [raw_x, raw_y, raw_z]
	
	@property
	def acceleration(self):
		# The accelerometer X, Y, Z axis values as a list of
		# m/s^2 values.
		raw = self.read_accel_raw()
		return map(lambda x: x * self._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD, raw)
	
	def read_mag_raw(self):
		# Read the raw magnetometer sensor values and return it as
		# a list of X, Y, Z axis values that are 16-bit unsigned values.
		# If you want the magnetometer in nice units, you probably want to
		# use the magnetometer property!
		
		# read the magnetometer
		buf = self._read_bytes(_MAGTYPE, _LSM9DS1_REGISTER_OUT_X_L_M, 6)
		raw_x = (buf[1] << 8) + buf[0]
		raw_x = _twos_comp(raw_x, 16)
		raw_y = (buf[3] << 8) + buf[2]
		raw_y = _twos_comp(raw_y, 16)
		raw_z = (buf[5] << 8) + buf[4]
		raw_z = _twos_comp(raw_z, 16)
		return [raw_x, raw_y, raw_z]
	
	@property
	def magnetic(self):
		# The magnetometer X, Y, Z axis values as a list of
		# gauss values.
		raw = self.read_mag_raw()
		corrected = [i-j for i,j in zip(raw, self.m_offset)]
		return map(lambda x: x * self._mag_mgauss_lsb / 1000.0, corrected)
	
	def read_gyro_raw(self):
		# Read the raw gyroscope sensor values and return it as
		# a list of X, Y, Z axis values that are 16-bit unsigned values.
		# If you want the gyroscope in nice units, you probably want to 
		# use the gyroscope property!
		
		# Read the gyroscope
		buf = self._read_bytes(_XGTYPE, _LSM9DS1_REGISTER_OUT_X_L_G, 6)
		raw_x = (buf[1] << 8) + buf[0]
		raw_x = _twos_comp(raw_x, 16)
		raw_y = (buf[3] << 8) + buf[2]
		raw_y = _twos_comp(raw_y, 16)
		raw_z = (buf[5] << 8) + buf[4]
		raw_z = _twos_comp(raw_z, 16)
		return [raw_x, raw_y, raw_z]
	
	@property
	def gyro(self):
		# The gyroscope X, Y, Z axis as a 3-tuple of
		# degrees/second values.
		raw = self.read_gyro_raw()
		return map(lambda x: x * self._gyro_dps_digit, raw)
	
	def read_temp_raw(self):
		# Read the raw temperature sensor value and return it as a
		# 12-bit signed value. If you want the temperature in nice units,
		# you probably want to use the temperature property!
		
		# read temp sensor
		buf = self._read_bytes(_XGTYPE, _LSM9DS1_REGISTER_TEMP_OUT_L, 2)
		temp = ((buf[1] << 8) | (buf[0])) >> 4
		return (_twos_comp(temp,12))
	
	@property
	def temperature(self):
		# The temperature of the sensor in degrees Celsius.
		temp = self.read_temp_raw()
		temp = 27.5 + temp/16
		return temp
	
	''' Determines offset parameters for magnetometer
		Roomba/IMU should be spinning when this function is called.
		Make sure Roomba/IMU spins 2-3 times during calibration. '''
	def CalibrateMag(self):
		# Initial max and min values
		self.m_offset = [0.0, 0.0, 0.0]
		[mx,my,mz] = self.magnetic # Read in magnetometer data
		x_min = mx
		x_max = mx
		y_min = my
		y_max = my
		z_min = mz
		z_max = mz
		basetime = time.time()
		while (time.time() - basetime) < 20.0: # Collect data for 20 seconds
			[mx,my,mz] = self.magnetic # Read new magnetometer data
			# Determine max and min values for each component
			if mx < x_min:
				x_min = mx
			elif mx > x_max:
				x_max = mx
			
			if my < y_min:
				y_min = my
			elif my > y_max:
				y_max = my
			
			if mz < z_min:
				z_min = mz
			elif mz > z_max:
				z_max = mz

		# Calculate offset parameters for each component
		self.m_offset = [(x_min + x_max)/2, (y_min + y_max)/2, (z_min + z_max)/2]
	
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
		return i2c.read_from(address)
	
	def _read_bytes(self, sensor_type, address, count):
		# Read a count number of bytes into a buffer starting from the
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
			buffer = []
			for x in range(0,count):
				buffer.append(i2c.read_from(current_addr)) # adds new int to end of list
				current_addr += 0x01
		return buffer
	
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

##################################################################
## Old LSM9DS1 IMU Class ##
class LSM9DS1_IMU:
	''' Initialization stuff for the IMU
		'''
	def __init__(self):
		# Initalize inertial measurement unit
		path = "LSM9DS1_RaspberryPi_Library/lib/liblsm9ds1cwrapper.so"
		# Make sure LSM9DS1 folder is in the same directory
		self.lib = cdll.LoadLibrary(path)
		# Checks if part of IMU is "available" (i.e., can give data)
		self.lib.lsm9ds1_gyroAvailable.argtypes = [c_void_p]
		self.lib.lsm9ds1_gyroAvailable.restype = c_int
		self.lib.lsm9ds1_accelAvailable.argtypes = [c_void_p]
		self.lib.lsm9ds1_accelAvailable.restype = c_int
		self.lib.lsm9ds1_magAvailable.argtypes = [c_void_p]
		self.lib.lsm9ds1_magAvailable.restype = c_int
		# Reads the values from parts of the IMU
		self.lib.lsm9ds1_readGyro.argtypes = [c_void_p]
		self.lib.lsm9ds1_readGyro.restype = c_int
		self.lib.lsm9ds1_readAccel.argtypes = [c_void_p]
		self.lib.lsm9ds1_readAccel.restype = c_int
		self.lib.lsm9ds1_readMag.argtypes = [c_void_p]
		self.lib.lsm9ds1_readMag.restype = c_int
		# Reads individual components of gyroscope after getting the values
		self.lib.lsm9ds1_getGyroX.argtypes = [c_void_p]
		self.lib.lsm9ds1_getGyroX.restype = c_float
		self.lib.lsm9ds1_getGyroY.argtypes = [c_void_p]
		self.lib.lsm9ds1_getGyroY.restype = c_float
		self.lib.lsm9ds1_getGyroZ.argtypes = [c_void_p]
		self.lib.lsm9ds1_getGyroZ.restype = c_float
		# Reads inidividual components of accelerometer after getting the values
		self.lib.lsm9ds1_getAccelX.argtypes = [c_void_p]
		self.lib.lsm9ds1_getAccelX.restype = c_float
		self.lib.lsm9ds1_getAccelY.argtypes = [c_void_p]
		self.lib.lsm9ds1_getAccelY.restype = c_float
		self.lib.lsm9ds1_getAccelZ.argtypes = [c_void_p]
		self.lib.lsm9ds1_getAccelZ.restype = c_float
		# Reads inidividual components of magnetometer after getting the values
		self.lib.lsm9ds1_getMagX.argtypes = [c_void_p]
		self.lib.lsm9ds1_getMagX.restype = c_float
		self.lib.lsm9ds1_getMagY.argtypes = [c_void_p]
		self.lib.lsm9ds1_getMagY.restype = c_float
		self.lib.lsm9ds1_getMagZ.argtypes = [c_void_p]
		self.lib.lsm9ds1_getMagZ.restype = c_float
		# Calculates human-readable values for each part of the IMU
		self.lib.lsm9ds1_calcGyro.argtypes = [c_void_p, c_float]
		self.lib.lsm9ds1_calcGyro.restype = c_float
		self.lib.lsm9ds1_calcAccel.argtypes = [c_void_p, c_float]
		self.lib.lsm9ds1_calcAccel.restype = c_float
		self.lib.lsm9ds1_calcMag.argtypes = [c_void_p, c_float]
		self.lib.lsm9ds1_calcMag.restype = c_float
		# Initializes IMU in the program
		self.imu = self.lib.lsm9ds1_create()
		self.lib.lsm9ds1_begin(self.imu)
		if self.lib.lsm9ds1_begin(self.imu) == 0: # If its not there...
			print("Failed to communicate with LSM9DS1.") # Something went wrong
			quit()
		# self.lib.lsm9ds1_calibrate(self.imu) # CHANGE BACK !!!!!!!!!!!!!!!!!!
		# Calibration offset variables
		self.mx_offset = 0.0
		self.my_offset = 0.0
		self.mz_offset = 0.0
		self.ax_offset = 0.0
		self.ay_offset = 0.0
		self.az_offset = 0.0
		self.gx_offset = 0.0
		self.gy_offset = 0.0
		self.gz_offset = 0.0
		# Calibration transformation matrix
		self.Atran = np.identity(3)

	## IMU Functions/Methods ##
	''' Read X, Y, and Z components of magnetometer
		Returns:
			cmx = float; x-value of magnetometer (Gauss)
			cmy = float; y-value of magnetometer (Gauss)
			cmz = float; z-value of magnetometer (Gauss) '''
	def ReadMagRaw(self):
		while self.lib.lsm9ds1_magAvailable(self.imu) == 0:
			pass # Wait for a value to be read from the magnetometer
		self.lib.lsm9ds1_readMag(self.imu)
		# Get the magnetometer components from the IMU
		mx = self.lib.lsm9ds1_getMagX(self.imu)
		my = self.lib.lsm9ds1_getMagY(self.imu)
		mz = self.lib.lsm9ds1_getMagZ(self.imu)
		# Calculate values for the magnetometer components
		cmx = self.lib.lsm9ds1_calcMag(self.imu, mx)
		cmy = self.lib.lsm9ds1_calcMag(self.imu, my)
		cmz = self.lib.lsm9ds1_calcMag(self.imu, mz)
		# Return magnetometer component values
		return [cmx,cmy,cmz]

	''' Determines offset parameters for magnetometer
		Roomba/IMU should be spinning when this function is called.
		Make sure Roomba/IMU spins 2-3 times during calibration. '''
	def CalibrateMag(self):
		# Initial max and min values
		[cmx,cmy,cmz] = self.ReadMagRaw() # Read in uncorrected magnetometer data
		x_min = cmx
		x_max = cmx
		y_min = cmy
		y_max = cmy
		z_min = cmz
		z_max = cmz
		basetime = time.time()
		while (time.time() - basetime) < 20.0: # Collect data for 20 seconds
			[cmx,cmy,cmz] = self.ReadMagRaw() # Read in uncorrected magnetometer data
			# Determine max and min values for each component
			if cmx < x_min:
				x_min = cmx
			if cmx > x_max:
				x_max = cmx
			if cmy < y_min:
				y_min = cmy
			if cmy > y_max:
				y_max = cmy
			if cmz < z_min:
				z_min = cmz
			if cmz > z_max:
				z_max = cmz

		# Calculate offset parameters for each component
		self.mx_offset = (x_min + x_max)/2
		self.my_offset = (y_min + y_max)/2
		self.mz_offset = (z_min + z_max)/2

	''' Read X, Y, and Z components of Magnetometer
		Returns:
			cmx - self.mx_offset = float; corrected x-value of magnetometer (Gauss)
			cmy - self.my_offset = float; corrected y-value of magnetometer (Gauss)
			cmz - self.mz_offset = float; corrected z-value of magnetometer (Gauss) '''
	def ReadMag(self):
		while self.lib.lsm9ds1_magAvailable(self.imu) == 0:
			pass # Wait for a value to be read from the magnetometer
		self.lib.lsm9ds1_readMag(self.imu)
		# Get the magnetometer components from the IMU
		mx = self.lib.lsm9ds1_getMagX(self.imu)
		my = self.lib.lsm9ds1_getMagY(self.imu)
		mz = self.lib.lsm9ds1_getMagZ(self.imu)
		# Calculate values for the magnetometer components
		cmx = self.lib.lsm9ds1_calcMag(self.imu, mx)
		cmy = self.lib.lsm9ds1_calcMag(self.imu, my)
		cmz = self.lib.lsm9ds1_calcMag(self.imu, mz)
		# Return offset magnetometer component values
		return [(cmx - self.mx_offset), (cmy - self.my_offset), (cmz - self.mz_offset)]

	''' Calculates cardinal heading in degrees from Magnetometer data
		Reads in the data from the magnetometer to determine the heading
		Returns:
			yaw = float; cardinal direction from magnetic North (degrees)
				(i.e., North = 0 (360); East = 90; South = 180; West = 270
		Important that you calibrate the magnetometer first. '''
	def CalculateHeading(self):
		[mx,my,mz] = self.ReadMag() # Get magnetometer x and y values (don't use z values)
		yaw = (math.degrees(math.atan2(-my,-mx))) # Calculate heading
		if yaw < 0: # Normalize heading value to [0,360)
			yaw += 360
		return yaw

	''' Calculates cardinal heading in degrees from given x, y magnetometer values
		Saves time by passing in magnetometer values to the function
		Parameters:
			mx = float; x value from the magnetometer
			my = float; y value from the magnetometer
		Returns:
			yaw = float; cardinal direction from magnetic North (degrees)
				(i.e., North = 0 (360); East = 90; South = 180; West = 270
		Important that you calibrate the magnetometer first. '''
	def CalculateHeadingXY(self, mx, my):
		yaw = (math.degrees(math.atan2(-my,-mx))) # Calculate heading
		if yaw < 0: # Normalize heading value to [0,360)
			yaw += 360
		return yaw

	''' Read X, Y, and Z components of accelerometer
		Returns:
			cax = float; x-value of accelerometer (g)
			cay = float; y-value of accelerometer (g)
			caz = float; z-value of accelerometer (g) '''
	def ReadAccelRaw(self):
		while self.lib.lsm9ds1_accelAvailable(self.imu) == 0:
			pass # Wait for a value to be read from the accelerometer
		self.lib.lsm9ds1_readAccel(self.imu)
		# Get the accelerometer components from the IMU
		ax = self.lib.lsm9ds1_getAccelX(self.imu)
		ay = self.lib.lsm9ds1_getAccelY(self.imu)
		az = self.lib.lsm9ds1_getAccelZ(self.imu)
		# Calculate values for the accelerometer components
		cax = self.lib.lsm9ds1_calcAccel(self.imu, ax)
		cay = self.lib.lsm9ds1_calcAccel(self.imu, ay)
		caz = self.lib.lsm9ds1_calcAccel(self.imu, az)
		# Return accelerometer component values
		return [cax,cay,caz]

	''' Read X, Y, and Z components of gyroscope
		Returns:
			cgx = float; x-value of gyroscope (degrees per second)
			cgy = float; y-value of gyroscope (degrees per second)
			cgz = float; z-value of gyroscope (degrees per second) '''
	def ReadGyroRaw(self):
		while self.lib.lsm9ds1_gyroAvailable(self.imu) == 0:
			pass # Wait for a value to be read from the gyroscope
		self.lib.lsm9ds1_readGyro(self.imu)
		# Get the gyroscope components from the IMU
		gx = self.lib.lsm9ds1_getGyroX(self.imu)
		gy = self.lib.lsm9ds1_getGyroY(self.imu)
		gz = self.lib.lsm9ds1_getGyroZ(self.imu)
		# Calculate values for the gyroscope components
		cgx = self.lib.lsm9ds1_calcGyro(self.imu, gx)
		cgy = self.lib.lsm9ds1_calcGyro(self.imu, gy)
		cgz = self.lib.lsm9ds1_calcGyro(self.imu, gz)
		# Return gyroscope component values
		return [cgx,cgy,cgz]

	''' Determines offset parameters for accelerometer and gyroscope
		Roomba/IMU should be still when this is called '''
	def CalibrateAccelGyro(self):
		# Calculate average value of accelerometer and gyroscope components
		ax_avg = 0
		ay_avg = 0
		az_avg = 0
		gx_avg = 0
		gy_avg = 0
		gz_avg = 0
		for i in range(0,1000): # Average 1000 readings
			[cax,cay,caz] = self.ReadAccelRaw() # Read in uncorrected accelerometer data
			ax_avg = (cax + (i * ax_avg))/(i+1)
			ay_avg = (cay + (i * ay_avg))/(i+1)
			az_avg = (caz + (i * az_avg))/(i+1)
			[cgx,cgy,cgz] = self.ReadGyroRaw() # Read in uncorrected gyroscope data
			gx_avg = (cgx + (i * gx_avg))/(i+1)
			gy_avg = (cgy + (i * gy_avg))/(i+1)
			gz_avg = (cgz + (i * gz_avg))/(i+1)
		# Average value over many data points is the offset value
		self.ax_offset = ax_avg
		self.ay_offset = ay_avg
		self.az_offset = (az_avg - 1) # Assumes z-axis is up
		# Calculate change of basis matrix for accelerometer values
		v3 = np.array([ax_avg, ay_avg, az_avg])
		v2 = np.array([-v3[0]*v3[1], pow(v3[0],2) + pow(v3[2],2), -v3[1]*v3[2]])
		v1 = np.array([v3[2], 0, -v3[0]])
		# Orthogonal basis vectors (all same length as v3)
		v3 = (v3/math.sqrt(np.dot(v3,v3)))*math.sqrt(np.dot(v3,v3)) # Technically redundant
		v2 = (v2/math.sqrt(np.dot(v2,v2)))*math.sqrt(np.dot(v3,v3))
		v1 = (v1/math.sqrt(np.dot(v1,v1)))*math.sqrt(np.dot(v3,v3))
		
		self.Atran = np.array([v1,v2,v3]) # Change of basis matrix (for row vectors)
		
		g1 = np.array([gx_avg, gy_avg, gz_avg])
		g_offset = np.matmul(g1,np.linalg.inv(self.Atran))
		# Need to check scaling of gyro offsets using change of basis matrix
		[self.gx_offset, self.gy_offset, self.gz_offset] = g_offset
	
	''' Read X, Y, and Z components of accelerometer
		Returns:
			cax - self.ax_offset = float; corrected x-value of accelerometer (g)
			cay - self.ay_offset = float; corrected y-value of accelerometer (g)
			caz - self.az_offset = float; corrected z-value of accelerometer (g) '''
	def ReadAccel(self):
		while self.lib.lsm9ds1_accelAvailable(self.imu) == 0:
			pass # Wait for a value to be read from the accelerometer
		self.lib.lsm9ds1_readAccel(self.imu)
		# Get the accelerometer components from the IMU
		ax = self.lib.lsm9ds1_getAccelX(self.imu)
		ay = self.lib.lsm9ds1_getAccelY(self.imu)
		az = self.lib.lsm9ds1_getAccelZ(self.imu)
		# Calculate values for the accelerometer components
		cax = self.lib.lsm9ds1_calcAccel(self.imu, ax)
		cay = self.lib.lsm9ds1_calcAccel(self.imu, ay)
		caz = self.lib.lsm9ds1_calcAccel(self.imu, az)
		w = np.array([cax,cay,caz])
		[tax,tay,taz] = np.matmul(w,np.linalg.inv(self.Atran)) # Matrix multiply with transformation matrix inverse
		# Return transformed accelerometer component values
		return [tax,tay,taz]

	''' Read X, Y, and Z components of gyroscope
		Returns:
			cgx - self.gx_offset = float; corrected x-value of gyroscope (degrees per second)
			cgy - self.gy_offset = float; corrected y-value of gyroscope (degrees per second)
			cgz - self.gz_offset = float; corrected z-value of gyroscope (degrees per second) '''
	def ReadGyro(self):
		while self.lib.lsm9ds1_gyroAvailable(self.imu) == 0:
			pass # Wait for a value to be read from the gyroscope
		self.lib.lsm9ds1_readGyro(self.imu)
		# Get the gyroscope components from the IMU
		gx = self.lib.lsm9ds1_getGyroX(self.imu)
		gy = self.lib.lsm9ds1_getGyroY(self.imu)
		gz = self.lib.lsm9ds1_getGyroZ(self.imu)
		# Calculate values for the gyroscope components
		cgx = self.lib.lsm9ds1_calcGyro(self.imu, gx)
		cgy = self.lib.lsm9ds1_calcGyro(self.imu, gy)
		cgz = self.lib.lsm9ds1_calcGyro(self.imu, gz)
		w = np.array([cgx,cgy,cgz])
		[tgx,tgy,tgz] = np.matmul(w,np.linalg.inv(self.Atran)) # Matrix multiply with change of basis matrix
		# Return transformed and offset gyroscope component values
		return [tgx - self.gx_offset, tgy - self.gy_offset, tgz - self.gz_offset]

##################################################################
## iRobot Create 2 (Roomba) Class ##
class Create_2:
	ddPin = 0 # Integer specifying ddPin placement on Raspberry Pi
		# Must be set after creating method object

	# Dictionary of packet byte size and sign
		# False = unsigned; True = signed;
	packet_dict = {
		7: [1, False], # Bump and Wheel Drops
		8: [1, False], # Wall (Light Bumper - Right)
		9: [1, False], # Cliff Left
		10: [1, False], # Cliff Front Left
		11: [1, False], # Cliff Front Right
		12: [1, False], # Cliff Right
		13: [1, False], # Virtual Wall
		14: [1, False], # Wheel Overcurrents
		15: [1, False], # Dirt Detect
		16: [1, False], # Unused Byte
		17: [1, False], # Infrared Character Omni
		18: [1, False], # Buttons
		19: [2, True], # Distance
		20: [2, True], # Angle
		21: [1, False], # Charging State
		22: [2, False], # Battery Voltage
		23: [2, True], # Battery Current
		24: [1, True], # Battery Temperature
		25: [2, False], # Battery Charge
		26: [2, False], # Battery Capacity
		27: [2, False], # Wall Signal (Light Bump Right Signal)
		28: [2, False], # Cliff Left Signal
		29: [2, False], # Cliff Front Left Signal
		30: [2, False], # Cliff Front Right Signal
		31: [2, False], # Cliff Right Signal
		32: [2, False], # Unused
		33: [1, False], # Unused
		34: [1, False], # Charging Sources Available
		35: [1, False], # OI Mode
		36: [1, False], # Song Number
		37: [1, False], # Song Playing
		38: [1, False], # Number of Stream Packets
		39: [2, True], # Requested Velocity
		40: [2, True], # Requested Radius
		41: [2, True], # Requested Right Velocity
		42: [2, True], # Requested Left Velocity
		43: [2, True], # Left Encoder Counts
		44: [2, True], # Right Encoder Counts
		45: [1, False], # Light Bumper
		46: [2, False], # Light Bump Left Signal
		47: [2, False], # Light Bump Front Left Signal
		48: [2, False], # Light Bump Center Left Signal
		49: [2, False], # Light Bump Center Right Signal
		50: [2, False], # Light Bump Front Right Signal
		51: [2, False], # Light Bump Right Signal
		52: [1, False], # Infrared Character Left
		53: [1, False], # Infrared Character Right
		54: [2, True], # Left Motor Current
		55: [2, True], # Right Motor Current
		56: [2, True], # Main Brush Motor Current
		57: [2, True], # Side Brush Motor Current
		58: [1, False], # Stasis
	}
	# Dictionary of packet group byte size and packet ID list; UNTESTED
	packet_group_dict = {
		0: [26, list(range(7,27))],
		1: [10, list(range(7,17))],
		2: [6, list(range(17,21))],
		3: [10, list(range(21,27))],
		4: [14, list(range(27,35))],
		5: [12, list(range(35,43))],
		6: [52, list(range(7,43))],
		100: [80, list(range(7,59))],
		101: [28, list(range(43,59))],
		106: [12, list(range(46,52))],
		107: [9, list(range(54,59))]
	}

	''' Initialization stuff for the Create_2
		Parameters:
			port = string; Raspberry Pi port location (Ex: "/dev/ttyS0")
			baud = integer; Roomba communication baud rate (Ex: 115200) '''
	def __init__(self, port, baud):
		self.conn = serial.Serial(port, baud) # Define serial port connection

	## Roomba Functions ##
	''' Send a single byte command directly to the Roomba
		Only does integers in range [0, 255] '''
	def DirectWrite(self, num):
		self.conn.write((num).to_bytes(1, byteorder='big', signed=False))

	''' Returns the number of bytes available to read from the buffer
		Returns:
			integer; number of bytes currently in the buffer '''
	def Available(self):
		return self.conn.inWaiting()

	''' Returns the bytes in the buffer in the order they were received
		Parameters:
			num = integer; number of bytes to read from the buffer
		Returns:
			bytes; byte sequence of length num '''
	def DirectRead(self, num):
		return self.conn.read(num)
		# Ex: 'Roomba.DirectRead(Roomba.Available()).decode()'; or maybe 'Roomba.conn.read(Roomba.conn.inWaiting())'

	''' Roomba Wake-up Sequence
		Parameters:
			control = integer; Control Command
			131 = Safe Mode; 132 = Full Mode (Be ready to catch it!) '''
	def WakeUp(self, control):
		self.conn.write(b'\x07') # Restart Roomba (7)
		time.sleep(8) # wait 8 seconds before continuing
		self.conn.write(b'\x80') # START command (128)
		time.sleep(1)
		self.DirectWrite(control) # Control command
		# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
		time.sleep(0.1)

	''' Roomba Shut-down Sequence
		Run at end of code to completely close Create_2 connection '''
	def ShutDown(self):
		self.conn.write(b'\x80') # Send Roomba to Passive Mode (128)
		self.conn.write(b'\xad') # Stop Roomba OI (173)
		time.sleep(0.05)
		self.conn.close() # Close the Roomba serial port.

	''' Blinks the clean button on Roomba during startup
		Helps determine that RPi -> Roomba communication is working '''
	def BlinkCleanLight(self):
		#Syntax: [139] [LED code] [LED color] [LED Intesnity]
		# Turn on Dirt Detect light and Green Clean button
		self.conn.write(b'\x8b\x19\x00\x80') # 139, 25, 0, 128
		time.sleep(0.5)
		# Change green to red
		self.conn.write(b'\x8b\x19\xff\x80') # 139, 25, 255, 128
		time.sleep(0.5)
		# Turn off Clean button
		self.conn.write(b'\x8b\x19\xff\x00') # 139, 25, 255, 0
		time.sleep(0.05)

	''' Send command to Roomba to move
		Parameters:
			x = integer; common wheel speed (mm/s); x > 0 -> forward motion
			y = integer; diffential wheel speed (mm/s); y > 0 -> CW motion
		Error may result if |x| + |y| > 500. '''
	def Move(self, x, y):
		RW = x - y # Right wheel speed
		LW = x + y # Left wheel speed
		self.conn.write(b'\x91') # Send command to Roomba to set wheel speeds (145)
		self.conn.write((RW).to_bytes(2, byteorder='big', signed=True))
		self.conn.write((LW).to_bytes(2, byteorder='big', signed=True))

	''' Request and Return a single Roomba sensor packet
		Parameters:
			packetID = integer; ID number for Roomba sensor packet
		Returns:
			integer; Value of the requested Roomba sensor packet
		Do not request the same packet more than once every 15.625 ms. '''
	def QuerySingle(self, packetID):
		self.conn.write(b'\x8e') # Request single sensor packet (142)
		self.DirectWrite(packetID) # Send packet ID of requested sensor

		while self.conn.inWaiting() == 0:
			pass # Wait for sensor packet values to be returned

		byte, sign = self.packet_dict[packetID] # Get packet info
		# Return the value of the requested packet
		return int.from_bytes(self.conn.read(byte), byteorder='big', signed=sign)

	''' Request and Return a list of Roomba sensor packets
		Combination of SendQuery() and Read(Query)
		Parameters:
			*args = integers; sequence of integers representing ID number of Roomba sensor packets
		Returns:
			integer list; List of values of requested packets, given in the order as requested
		Do not request the same set of packets more than once every 15.625 ms. '''
	def Query(self, *args):
		num = len(args) # Find number of packets to request
		self.conn.write(b'\x95') # Request sensor query (149)
		self.DirectWrite(num) # Number of packets to request
		for packetID in args:
			self.DirectWrite(packetID) # Send packet ID for each sensor packet to request

		while self.conn.inWaiting() == 0:
			pass # Wait for sensor packet values to be returned

		data = [] # Create empty list
		for packetID in args:
			byte, sign = self.packet_dict[packetID] # Get packet info
			# Add the sensor value to the list
			data.append(int.from_bytes(self.conn.read(byte), byteorder='big', signed=sign))
		return data # Return the list of values

	''' Request a list of Roomba sensor packets
		Parameters:
			*args = integers; sequence of integers representing ID number of Roomba sensor packets
		Do not request the same set of packets more than once every 15.625 ms. '''
	def SendQuery(self, *args):
		num = len(args) # Find number of packets to request
		self.conn.write(b'\x95') # Request sensor query (149)
		self.DirectWrite(num) # Number of packets to request
		for packetID in args:
			self.DirectWrite(packetID) # Send packet ID for each sensor packet to request

	''' Return a list of Roomba sensor packets
		Parameters:
			*args = integers; sequence of integers representing ID number of Roomba sensor packets
				Needs to be the same sequence as used in SendQuery()
		Returns:
			integer list; List of values of requested packets, given in the order as requested
		Make sure the bytes are in the Roomba buffer before calling this function'''
	def ReadQuery(self, *args):
		data = [] # Create empty list
		for packetID in args:
			byte, sign = self.packet_dict[packetID] # Get packet info
			# Add the sensor value to the list
			data.append(int.from_bytes(self.conn.read(byte), byteorder='big', signed=sign))
		return data # Return the list of values

	''' Starts Data Stream with specified packets
		Parameters:
			*args = integers; sequence of integers representing ID number of Roomba sensor packets
		'''
	def StartQueryStream(self, *args):
		num = len(args)
		self.conn.write(b'\x94') # Start sensor data stream (148)
		self.DirectWrite(num) # Number of packets to stream
		for packetID in args:
			self.DirectWrite(packetID) # Send packet ID for each sensor packet to request

	''' Reads in data from Roomba sent as a Query Stream
		The sequence of bytes is very structured
		Parameters:
			*args = integers; sequence of integers representing ID numbers of Roomba sensor packets
		Returns:
			integer list; List of values of requested packets, given in the order as requested
				Returns a zero for any packet that is requested, but not sent from the Roomba
		Do not call this function until you have started or resumed the Query Stream
		Make sure data is in the Roomba buffer before calling this function
		Roomba updates stream every 15.625 ms '''
	def ReadQueryStream(self, *args):
		value_dict = {} # Empty dictionary to store packet values

		header = 0 # Initial the header value
		while header != 19: # Continues when the header byte is found; Error may result if a packet value is 19
			# Read in the header byte value
			header = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)

		# Number of bytes to read (not counting checksum)
		num = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)
		check = 19 + num # cummulative total of all byte values

		while num > 0: # Go until we reach the checksum
			# The first byte is the packet ID
			packetID = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)
			byte, sign = self.packet_dict[packetID] # Get packet info
			# Determine the value of the current packet
			value = int.from_bytes(self.conn.read(byte), byteorder='big', signed=sign)
			value_dict[packetID] = value # Create new entry in the dictionary
			# Calculate byte sum of the packet ID and value
			check += packetID # Add in the packet ID value
			buffer = (value).to_bytes(byte, byteorder='big', signed=sign)
			byte_value = int.from_bytes(buffer, byteorder='big', signed=False) # Convert to unsigned integer
			while byte_value > 255:
				check += (byte_value % 256) # Value of low byte
				byte_value = (byte_value // 256) # Value of high byte
			check += byte_value # Add in packet value

			num -= (byte + 1) # Update the number of bytes left to read

		# Read the check sum value
		checksum = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)
		check += checksum
		if (check % 256) != 0: # If checksum isn't correct
			value_dict.clear() # Erase dictionary; it is corrupted

		# return data found
		data = [] # Create empty list
		for packetID in args:
			# Add the sensor value to the list
			data.append(value_dict.get(packetID, 0))
			# Returns a zero if a packetID is not found

		return data # Return the list of values


	''' Pause the current Query Stream
		Does not erase the last set of packets requested '''
	def PauseQueryStream(self):
		self.conn.write(b'\x96\x00') # Pause the Roomba data stream (150, 0)

	''' Resume the current Query Stream
		Uses the last set of packets requested '''
	def ResumeQueryStream(self):
		self.conn.write(b'\x96\x01') # Resume the Roomba data stream (150, 1)

	''' You are not expected to understand this. :)
		'''
	def PlaySMB(self):
		#Define SMB Theme song
		self.conn.write(b'\x8c\x00\x0b') # 140, 0, 11
		self.conn.write(b'\x4c\x08\x4c\x0c\x1e\x04\x4c\x0c\x1e\x04\x48\x08\x4c\x0c\x1e\x04\x4f\x0c\x1e\x14\x43\x0c')
		#                   76,  8, 76, 12, 30,  4, 76, 12, 30,  4, 72,  8, 76, 12, 30,  4, 79, 12, 30, 20, 67, 12
		time.sleep(0.05)
		#Play song
		self.conn.write(b'\x8d\x00') # 141, 0
		time.sleep(2) # Wait for song to play

##################################################################
## Additional Functions ##
''' Returns Roomba spin amount to achieve desired heading set point
	Parameters:
		angle = float; Current direction of Roomba (in degrees)
			The value you are currenting "facing"
		desired_heading = float; Desired direction set point (in degrees)
			The value that you want to "face"
		epsilon = float; value of a buffer zone on either side of the set point (in degrees)
			Allows for reduced shaking when reaching the desired set point
	Returns:
		spin_value = int; The value needed to spin the Roomba from the current "angle" to the "desired_heading"
			The value is larger the farther from the set point
	'''
def DHTurn(angle, desired_heading, epsilon):
	# Threshold Constants
	thresh_1 = (50 * epsilon) # First threshold value (degrees)
	thresh_2 = (10 * epsilon) # Second threshold value (degrees)

	diff = abs(angle - desired_heading)
	# Determine spin speed based on thresholds
	if (diff > thresh_1 and diff < (360 - thresh_1)):
		spin_value = 100 # Move faster when farther away from the set point
	elif (diff > thresh_2 and diff < (360 - thresh_2)):
		spin_value = 50 # Move slower when closer to the set point
	else:
		spin_value = 15 # Move very slow when very close to the set point
		# Reduces oscillations due to magnetometer variation and loop execution rate
	
	# Determine direction of spin
	diff = angle - desired_heading
	# Normalize diff to [-0.5*360, 0.5*360]
	if diff > 180:
		diff -= 360
	elif diff < -180:
		diff += 360
	# Set spin direction
	if diff > epsilon:
		spin_dir = -1 # Spin CCW
	elif diff < -epsilon:
		spin_dir = 1 # Spin CW
	else:
		spin_dir = 0 # Don't spin
	
	return (spin_value * spin_dir)

''' Returns Roomba forward speed to make sure it can reach a certain set point
		in a direction of "desired_heading" a distance "distance" millimeters away
	Parameters:
		angle = float; Current direction of Roomba (in degrees)
			The value you are currenting "facing"
		desired_heading = float; Direction to the set point (in degrees)
			The value that you want to "face"
		distance = float; Straight-line distance to the set point
			The distance that you still need to go to get to the set point
	Returns:
		forward_value = int; forward speed for Roomba to travel to reach the set point
			The value is larger the farther away from the set point
			The value is zero if the set point is unreachable at the standard forward_value
	'''
def DDSpeed(angle, desired_heading, distance):
	# Threshold Constants
	spin_thresh_1 = 25 # First spin threshold value (degrees)
	spin_thresh_2 = 5 # Second spin threshold value (degrees)
	forward_thresh_1 = 100 # First forward threshold value (mm)

	diff = abs(angle - desired_heading)
	# Determine spin speed based on thresholds (Same as DHTurn())
	if (diff > spin_thresh_1 and diff < (360 - spin_thresh_1)):
		spin_value = 100 # Move faster when farther away from the set point
	elif (diff > spin_thresh_2 and diff < (360 - spin_thresh_2)):
		spin_value = 50 # Move slower when closer to the set point
	else:
		spin_value = 15 # Move very slow when very close to the set point
		# Reduces oscillations due to magnetometer variation and loop execution rate

	if distance > forward_thresh_1:
		forward_value = 100 # Move faster when farther away from the set point
	else:
		forward_value = 50 # Move slower when closer to the set point

	# Roomba Constants
	WHEEL_SEPARATION = 235 # millimeters

	# Radius of unreachable region at the given forward and spin values
	radius = WHEEL_SEPARATION * (((forward_value + spin_value)/(2 * spin_value)) - 0.5)
	# Minimum distance that can be guaranteed to be reached for the given forward and spin values
	d_min = radius * math.sqrt(2 * (1 - math.cos(math.radians(2*diff))))

	if (diff > 90 and diff < 270) and distance < (2 * radius): # Special case
		return 0 # Need to turn around to get to the close set point
	if d_min > distance: # Normal cases
		return 0 # Need to turn some to get to the close set point
	else:
		return forward_value # We can head forward to reach the set point
