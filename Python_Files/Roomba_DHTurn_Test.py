''' Roomba_DHTurn_Test.py
Purpose: Test DHTurn() function using IMU magnetometer data
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/6/2019
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

import RoombaCI_lib # Make sure this file is in the same directory
from RoombaCI_lib import DHTurn

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 57600
# LED pin numbers
yled = 5
rled = 6
gled = 13

# Timing Counter Parameters
data_timer = 0.2
reset_timer = 10

# DH_Turn Parameters
epsilon = 1.0 # (Ideally) smallest resolution of magnetometer
data_counter = 0 # Data number counter

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
Roomba.ddPin = 23
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
time.sleep(3) # Gives time to read offset values before continuing
GPIO.output(yled, GPIO.LOW) # Indicate setup sequence is complete

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging

GPIO.output(yled, GPIO.LOW) # Indicate setup sequence complete

# Main Code #
angle = imu.CalculateHeading() # Get initial heading information
forward = 0
desired_heading = 0
data_base = time.time()
reset_base = time.time()

while True:		
	try:
		# Update heading of Roomba
		angle = imu.CalculateHeading()
		
		spin = DHTurn(angle, desired_heading, epsilon) # Value needed to turn to desired heading point
		Roomba.Move(forward, spin) # Move Roomba to desired heading point
		
		if spin == 0:
			GPIO.output(yled, GPIO.LOW) # Indicate Roomba is not turning
		else:
			GPIO.output(yled, GPIO.HIGH) # Indicate Roomba is turning
		
		if (time.time() - reset_base) > reset_timer:
			desired_heading += 90
			if desired_heading >= 360:
				desired_heading -= 360
			reset_base += reset_timer
		
		# Print heading data to monitor every second
		if (time.time() - data_base) > data_timer: # After one second
			[mx,my,mz] = imu.magnetic # Read magnetometer component values
			angle = imu.CalculateHeading() # Calculate heading
			# Note: angle may not correspond to mx, my, mz
			#[ax,ay,az] = imu.acceleration # Read accelerometer component values
			#[gx,gy,gz] = imu.gyro # Read gyroscope component values
			
			print("{0:.4f}, {1:.4f}, {2:.5f}, {3:.5f}, {4:.5f}".format(angle,desired_heading,mx,my,mz))
			data_base += data_timer
				
	except KeyboardInterrupt:
		print('') # print new line
		break # exit while loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.Move(0,0) # Stop Roomba movement
#Roomba.PlaySMB()
GPIO.output(gled, GPIO.LOW) # Turn off green LED
GPIO.output(yled, GPIO.LOW) # Turn off yellow LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
