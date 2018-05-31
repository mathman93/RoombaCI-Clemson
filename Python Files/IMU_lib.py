''' IMU_lib.py
Purpose: Python Library with LSM9DS1 class and specific functions
Import this file in main Python file to access functions
Last Modified: 5/25/2018
'''
## Import Libraries ##
from ctypes import *
import time
import math
## LSM9DS1 IMU Class ##
class LSM9DS1_IMU:
	''' Initialization stuff for the IMU
		'''
	def __init__(self):
		# Initalize inertial measurement unit
		path = "../LSM9DS1_RaspberryPi_Library/lib/liblsm9ds1cwrapper.so"
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
		self.lib.lsm9ds1_calibrate(self.imu)
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
	
	# IMU Functions/Methods #
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
		x_min = 10.0
		x_max = -10.0
		y_min = 10.0
		y_max = -10.0
		z_min = 10.0
		z_max = -10.0
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
		Returns:
			yaw = float; cardinal direction from magnetic North (degrees)
				(i.e., North = 0 (360); East = 90; South = 180; West = 270
		Important that you calibrate the magnetometer first. '''
	def CalculateHeading(self):
		[mx,my,mz] = self.ReadMag() # Get magnetometer x and y values (don't uses z values)
		yaw = (math.degrees(math.atan2(my,mx))) # Calculate heading
		#yaw = (math.degrees(math.atan2(mx,-my))) - 90 # Alternate?
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
		for i in range(0,1000) # Average 1000 readings
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
		self.gx_offset = gx_avg
		self.gy_offset = gy_avg
		self.gz_offset = gz_avg
	
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
		# Return offset accelerometer component values
		return [(cax - self.ax_offset), (cay - self.ay_offset), (caz - self.az_offset)]
	
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
		# Return gyroscope component values
		return [(cgx - self.gx_offset), (cgy - self.gy_offset), (cgz - self.gz_offset)]
	