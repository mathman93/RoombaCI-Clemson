'''
StaticDistanceTest.py
Purpose: Estimate distance between lightbar sensors and an object
Last Modified: 10/25/2021
'''
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
	1: [60.0, 0, 50]
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
#print(" Calibrating IMU...")
#Roomba.Move(0,75) # Start Roomba spinning
#imu.CalibrateMag() # Calculate magnetometer offset values
#Roomba.Move(0,0) # Stop Roomba spinning
#time.sleep(0.5)
#imu.CalibrateGyro() # Calculate gyroscope offset values

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
time_base = time.time()
Roomba.StartQueryStream(46, 47, 48, 49, 50, 51) # Start query stream with specific sensor packets
start_time = time.time()
state = 0
frDist = 0
time_out = 10
while True:
	try:
		if Roomba.Available()>0:
			lb_ll, lb_fl, lb_cl, lb_cr, lb_fr, lb_rr = Roomba.ReadQueryStream( 46, 47, 48, 49, 50, 51)
			#Roomba.UpdatePosition(l_counts, r_counts)
			frDist =((1123-593)/(3609-9)) * lb_fr
			outString = "{0}, {1}, {2}, {3}, {4}, {5}, {6:.2f}".format( lb_ll, lb_fl, lb_cl, lb_cr, lb_fr, lb_rr, frDist)
			print(outString)
	except KeyboardInterrupt:
		break
#Roomba.Move(0,0) # Stop moving
Roomba.PauseQueryStream() # End Roomba Query Stream
if Roomba.Available()>0: # If data exists in Query Stream...
	z = Roomba.DirectRead(Roomba.Available()) # Clear out data
	print(z) # Include for debugging
# End if Roomba.Available()
# End if file_create

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program