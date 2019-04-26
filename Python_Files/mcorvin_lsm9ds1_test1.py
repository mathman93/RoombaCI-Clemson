# First test of the first draft of the new lsm9ds1 library.

import mcorvin_LSM9DS1 as lsm9ds1
import time

# Setup:
sensor = lsm9ds1.LSM9DS1_I2C()

# At this point the loop will repeat until 'Ctrl+C' is typed.
while True:
	# Read acceleration, magnetometer, gyroscope, temperature
	ax_r, ay_r, az_r = sensor.read_accel_raw()
	accel_x, accel_y, accel_z = sensor.acceleration
	mx_r, my_r, mz_r = sensor.read_mag_raw()
	mag_x, mag_y, mag_z = sensor.magnetic
	gx_r, gy_r, gz_r = sensor.read_gyro_raw()
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature
	
	# Print values
	print('Acceleration (raw): ({0:04X},{1:04X},{2:04X})'.format(ax_r, ay_r, az_r)) # Testing
	print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
	print('Magnetometer (raw): ({0:04X},{1:04X},{2:04X})'.format(mx_r, my_r, mz_r)) # Testing
	print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
	print('Gyroscope (raw): ({0:04X},{1:04X},{2:04X})'.format(gx_r, gy_r, gz_r)) # Testing
	print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
	print('Temperature: {0:0.3f}C'.format(temp))
	print('\n')
	
	#Delay
	time.sleep(1.0)
