# First test of the first draft of the new lsm9ds1 library.

import mcorvin_LSM9DS1 as lsm9ds1
import time

# Setup:
sensor = lsm9ds1.LSM9DS1_I2C()

while True:
    # Read acceleration, magnetometer, gyroscope, temperature
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    #temp = sensor.temperature

    # Print values
    print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
    print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    print('\n')
    #print('Temperature: {0:0.3f}C'.format(temp))

    #Delay
    time.sleep(1.0)
