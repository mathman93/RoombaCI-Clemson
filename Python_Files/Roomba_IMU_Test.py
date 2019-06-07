''' Roomba_IMU_Test.py
Purpose: Testing communication between Roomba and LSM9DS1 IMU
	Form basis of Roomba code for other tests.
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/6/2019
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import numpy as np
import math
import os.path
import RoombaCI_lib

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

data_counter = 0 # Initialize data_counter
#global A # Accelerometer transformation matrix

move_dict = {
	0: [2.0, 0, 0],
	1: [10.0, 75, 0],
	2: [2.0, 0, 0],
	3: [5.0, 0, 75],
	4: [2.0, 0, 0],
	5: [10.0, 75, 0],
	6: [2.0, 0, 0]
	}

## Functions and Definitions ##
''' Displays current date and time to the screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake Up Roomba Sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" Starting ROOMBA... ")
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
Roomba.ddPin = 23 # Set Roomba dd pin number
GPIO.setup(Roomba.ddPin, GPIO.OUT, initial=GPIO.LOW)
Roomba.WakeUp(131) # Start up Roomba in Safe Mode
# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
Roomba.BlinkCleanLight() # Blink the Clean light on Roomba

if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	#print(x) # Include for debugging

print(" ROOMBA Setup Complete")
GPIO.output(yled, GPIO.HIGH) # Indicate within setup sequence
# Initialize IMU
print(" Starting IMU...")
imu = RoombaCI_lib.LSM9DS1_I2C() # Initialize IMU
time.sleep(0.1)
# Clear out first reading from all sensors
x = imu.magnetic
x = imu.acceleration
x = imu.gyro
# Calibrate IMU
print(" Calibrating IMU...")
Roomba.Move(0,75) # Start Roomba spinning
imu.CalibrateMag() # Calculate magnetometer offset values
Roomba.Move(0,0) # Stop Roomba spinning
time.sleep(0.5)
imu.CalibrateGyro() # Calculate gyroscope offset values
# Display offset values
print("mx_offset = {:f}; my_offset = {:f}; mz_offset = {:f}"\
	.format(imu.m_offset[0], imu.m_offset[1], imu.m_offset[2]))
print("gx_offset = {:f}; gy_offset = {:f}; gz_offset = {:f}"\
	.format(imu.g_offset[0], imu.g_offset[1], imu.g_offset[2]))
print(" IMU Setup Complete")
#time.sleep(3) # Gives time to read offset values before continuing
GPIO.output(yled, GPIO.LOW) # Indicate setup sequence is complete

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging

# Main Code #
# Open a text file for data retrieval
file_name_input = input("Name for data file: ")
dir_path = "/home/pi/RoombaCI-Clemson/Data_Files/2019_Summer/" # Directory path to save file
file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
datafile = open(file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

basetime = time.time()
basetime_offset = (1/64)
Roomba.Move(0,0)

# Read in initial values
[r_speed,l_speed,l_counts,r_counts] = Roomba.Query(41,42,43,44) # Read Roomba data stream
data_time = 0.0
[ax,ay,az] = imu.acceleration # Read accelerometer component values
[gx,gy,gz] = imu.gyro # Read gyroscope component values
[mx,my,mz] = imu.magnetic # Read magnetometer component values
# Write data values to a text file
datafile.write("{0:.6f}, {1:.6f}, {2:.6f}, {3:.6f}, {4:.6f}, {5:.6f}, {6:.6f}, {7:.6f}, {8:.6f}, {9:.6f}, {10}, {11}, {12}, {13}\n"\
	.format(data_time, ax, ay, az, gx, gy, gz, mx, my, mz, l_speed, r_speed, l_counts, r_counts))
print("{0:.6f}, {1:.6f}, {2:.6f}, {3:.6f}, {4:.6f}, {5:.6f}, {6:.6f}, {7:.6f}, {8:.6f}, {9:.6f}"\
	.format(data_time, ax, ay, az, gx, gy, gz, mx, my, mz))

# Start up Roomba query stream
Roomba.StartQueryStream(41,42,43,44) # Start query stream with specific sensor packets
time_base = time.time() # Set data timer base

for i in range(0, len(move_dict.keys())):
	[movetime_offset, forward, spin] = move_dict[i] # Read values from dictionary
	Roomba.Move(forward, spin)
	movetime_base = time.time()
	while (time.time() - movetime_base) < movetime_offset:
		if Roomba.Available() > 0: # If data comes in from the Roomba
			# Retrieve data values (Happens every ~1/64 seconds)
			data_time = time.time() - time_base # Time that data is received
			[r_speed,l_speed,l_counts,r_counts] = Roomba.ReadQueryStream(41,42,43,44) # Read Roomba data stream
			[ax,ay,az] = imu.acceleration # Read accelerometer component values
			[gx,gy,gz] = imu.gyro # Read gyroscope component values
			[mx,my,mz] = imu.magnetic # Read magnetometer component values
			# Write data values to a text file
			datafile.write("{0:.6f}, {1:.6f}, {2:.6f}, {3:.6f}, {4:.6f}, {5:.6f}, {6:.6f}, {7:.6f}, {8:.6f}, {9:.6f}, {10}, {11}, {12}, {13}\n"\
				.format(data_time, ax, ay, az, gx, gy, gz, mx, my, mz, l_speed, r_speed, l_counts, r_counts))
			print("{0:.6f}, {1:.6f}, {2:.6f}, {3:.6f}, {4:.6f}, {5:.6f}, {6:.6f}, {7:.6f}, {8:.6f}, {9:.6f}"\
				.format(data_time, ax, ay, az, gx, gy, gz, mx, my, mz))
		# End if Roomba.Available() > 0
		
	# End while (time.time() - movetime_base) < movetime_offset
	
# End for i in range(1, len(move_dict.keys())+1)

Roomba.Move(0,0) # Stop Roomba movement
Roomba.PauseQueryStream() # Pause Query Stream before ending program
if Roomba.Available() > 0:
	x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
	#print(x) # Include for debugging purposes

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PlaySMB()
datafile.close()
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
