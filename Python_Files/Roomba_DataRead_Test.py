'''
Roomba_DataRead_Test.py
Purpose: Collect sesnor data and test communication between Roomba and RaspberryPi
	Form basis of Roomba code for other tests.
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 8/17/2021
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import os.path

import RoombaCI_lib

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

# Dictionary of Roomba movements
move_dict = {
	0: [1.0, 0, 0],
	1: [60.0, 0, 0]
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

ack = input("Do you want to save data to a file (y/n)? ") # user acknowledgement
if ack in ["y", "Y"]: # If positive
	file_create = True # create and save data to file
	file_name_input = input("Name for data file: ") # Ask user for desired file name
	dir_path = "/home/pi/RoombaCI-Clemson/Data_Files/2021_Fall/" # Directory path to save file
	file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
	datafile = open(file_name, "w") # Open a text file for storing data
		# Will overwrite anything that was in the text file previously
else: # otherwise
	file_create = False # Skip data creation
# End if

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
time.sleep(3.0) # Gives time to read offset values before continuing
GPIO.output(yled, GPIO.LOW) # Indicate setup sequence is complete

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging

# Main Code #
''' # Variables needed for Query and QuerySingle commands
query_time = time.time() # Set base time for query
query_time_offset = (5/64) # Set time offset for query
# smallest time offset for query is 15.625 ms.
'''

for_break = False # To break out of for loop on keyboard interrupt
Roomba.StartQueryStream(7, 43, 44, 45, 46, 47, 48, 49, 50, 51) # Start query stream with specific sensor packets
time_base = time.time()

for i in range(0, len(move_dict.keys())):
	[movetime_offset, forward, spin] = move_dict[i] # Read values from dictionary
	Roomba.Move(forward, spin)
	movetime_base = time.time()
	while (time.time() - movetime_base) < movetime_offset:
		try:
			'''# Request data packet from Roomba (QuerySingle)
			if (time.time() - query_time) > query_time_offset: # If enough time has passed
				bumper_byte = Roomba.QuerySingle(7) # Ask for Bumper byte packet (1 byte)
				print(bumper_byte)
				query_time += query_time_offset # offset query time for next query
			# End if
			'''
			'''# Request data packets from Roomba (Query)
			if (time.time() - query_time) > query_time_offset: # If enough time has passed
				bumper_byte, l_wheel, r_wheel, light_bumper = Roomba.Query(7, 43, 44, 45) # Ask for specific sensor packets
				# Print data to screen (MATLAB format)
				print("{0}, {1:0>8b}, {2}, {3}, {4:0>8b};".format(data_counter, bumper_byte, l_wheel, r_wheel, light_bumper))
				data_counter += 1 # Increment counter for the next data sample
				query_time += query_time_offset # offset query time for next query
			# End if
			'''
			# Read query stream for specific packets (ReadQueryStream)
			if Roomba.Available() > 0:
				data_time = time.time() - time_base
				bumper_byte, l_counts, r_counts, light_bumper, lb_ll, lb_fl, lb_cl, lb_cr, lb_fr, lb_rr = Roomba.ReadQueryStream(7, 43, 44, 45, 46, 47, 48, 49, 50, 51)
				angle = imu.CalculateHeading()
				# Print data values out to the monitor
				print("{0:.6f}, {1}, {2}, {3:.4f}, {4:0>8b}, {5:0>8b}, {6}, {7}, {8}, {9}, {10}, {11};"\
					.format(data_time, l_counts, r_counts, angle, bumper_byte, light_bumper, lb_ll, lb_fl, lb_cl, lb_cr, lb_fr, lb_rr))
				if file_create == True:
					# Write data values to a text file (MATLAB format)
					datafile.write("{0:.6f}, {1}, {2}, {3:.4f}, {4:0>8b}, {5:0>8b}, {6}, {7}, {8}, {9}, {10}, {11};\n"\
						.format(data_time, l_counts, r_counts, angle, bumper_byte, light_bumper, lb_ll, lb_fl, lb_cl, lb_cr, lb_fr, lb_rr))
				# End if
			# End if
		except KeyboardInterrupt:
			print('') # print new line
			for_break = True
			break # exit while loop
	# End while
	if for_break == True:
		break
	# End if
# End for

Roomba.Move(0,0) # Stop Roomba movement
Roomba.PauseQueryStream() # Pause Query Stream before ending program
if Roomba.Available() > 0:
	x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
	#print(x) # Include for debugging purposes

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
if file_create == True:
	datafile.close()
	Roomba.PlaySMB()
# End if
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
