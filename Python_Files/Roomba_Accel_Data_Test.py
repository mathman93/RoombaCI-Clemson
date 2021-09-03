''' Roomba_Accel_Data_Test.py
Purpose: Get Acceleration data as Roomba moves a set distance (NOT TESTED)
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 8/2/2021
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

import RoombaCI_lib
from RoombaCI_lib import DHTurn

import math

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

epsilon = 0.5 # smallest resolution of angle

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
time.sleep(3) # Gives time to read offset values before continuing
GPIO.output(yled, GPIO.LOW) # Indicate setup sequence is complete

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging

# Main Code #
# Get initial angle from IMU
Roomba.heading = math.radians(imu.CalculateHeading())

# Read in initial wheel count values from Roomba
bumper_byte, l_counts_current, r_counts_current, l_speed, r_speed, light_bumper = Roomba.Query(7, 43, 44, 42, 41, 45) # Read new data
Roomba.SetWheelEncoderCounts(l_counts_current, r_counts_current)

while True:
	try:
		forward_value = 0 # Initial forward speed (millimeters/second)
		# Print current angle of Roomba
		print("Current Angle: {:.3f}".format(math.degrees(Roomba.heading)))
		# Request for the desired angle to turn to
		desired_heading = float(input("New Angle (in degrees)? ")) # in degrees

		# Start Query Data Stream
		Roomba.StartQueryStream(7,43,44,42,41,45)
		#data_time = 0.0 # 0 seconds initial
		# Determine initial spin speed value for Roomba
		spin_value = DHTurn(math.degrees(Roomba.heading), desired_heading, epsilon)
		# Restart base timers
		base = time.time()

		while spin_value != 0: # If the Roomba needs to turn...
			try:

				if Roomba.Available() > 0:
					# Record the current time since the beginning of loop
					data_time = time.time() - base

					bumper_byte, l_counts, r_counts, l_speed, r_speed, light_bumper = Roomba.ReadQueryStream(7,43,44,42,41,45) # Read new data
					[mx,my,mz] = imu.ReadMag() # Read magnetometer component values
					#imu_angle = imu.CalculateHeading() # Calculate heading
					# Note: imu_angle may not correspond to mx, my, mz
					[ax,ay,az] = imu.ReadAccel() # Read accelerometer component values
					[gx,gy,gz] = imu.ReadGyro() # Read gyroscope component values

					Roomba.UpdatePosition(l_counts, r_counts)
					
					spin_value = DHTurn(math.degrees(Roomba.heading), desired_heading, epsilon) # Determine the spin speed to turn toward the desired heading

					# Print out pertinent data values
					#print("{0:.5f}, {1:.3f}, {2:.3f}, {3:.3f}, {4:.3f}, {5:0>8b}, {6:0>8b}, {7}, {8};".format(data_time,desired_heading,math.degrees(Roomba.heading),Roomba.Y_position,Roomba.X_position,bumper_byte,light_bumper,l_counts,r_counts))
					print("{0:.5f}, {1:.5f}, {2:.5f}, {3:.5f}, {4:.5f}, {5:.5f}, {6:.5f}, {7:.5f}, {8:.5f}, {9:.5f};".format(data_time,mx,my,mz,ax,ay,az,gx,gy,gz))

					Roomba.Move(forward_value, spin_value) # Spin the Roomba toward the desired heading

				# End if
			except KeyboardInterrupt:
				break
			# End try
		# End while spin_value
		forward_value = 0 # initial forward speed value (mm/s)
		spin_value = 0 # initial spin speed value (mm/s)
		Roomba.Move(forward_value, spin_value) # Stop Roomba movement
		Roomba.PauseQueryStream() # Pause Query Stream before ending program
		if Roomba.Available() > 0:
			x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
			#print(x) # Include for debugging purposes
		# End if
		Roomba.total_distance = 0.0 # reset initial distance travelled
		# Request amount of distance to travel
		desired_distance = float(input("Distance (in mm)? ")) # in millimeters
		# Request amount of speed to travel
		forward_value = int(input("Speed (in mm/s)? ")) # in millimeters per second

		Roomba.ResumeQueryStream()
		# Restart base timers
		base = time.time()
		#query_base = time.time()

		while Roomba.total_distance < desired_distance: # Until we have reached the desired_distance...
			try:
				#print("Testing 2nd Loop") # Include for debugging

				if Roomba.Available() > 0:
					# Record the current time since the beginning of loop
					data_time = time.time() - base

					bumper_byte, l_counts, r_counts, l_speed, r_speed, light_bumper = Roomba.ReadQueryStream(7,43,44,42,41,45) # Read new wheel counts
					[mx,my,mz] = imu.magnetic # Read magnetometer component values
					#imu_angle = imu.CalculateHeading() # Calculate heading
					# Note: imu_angle may not correspond to mx, my, mz
					[ax,ay,az] = imu.accelerometer # Read accelerometer component values
					[gx,gy,gz] = imu.gyro # Read gyroscope component values

					Roomba.UpdatePosition(l_counts, r_counts)

					spin_value = DHTurn(math.degrees(Roomba.heading), desired_heading, epsilon) # Determine the spin speed to turn toward the desired heading

					# Print out pertinent data values
					#print("{0:.5f}, {1:.3f}, {2:.3f}, {3:.3f}, {4:.3f}, {5:0>8b}, {6:0>8b}, {7}, {8};".format(data_time,desired_heading,math.degrees(Roomba.heading),Roomba.Y_position,Roomba.X_position,bumper_byte,light_bumper,l_counts,r_counts))
					print("{0:.5f}, {1:.5f}, {2:.5f}, {3:.5f}, {4:.5f}, {5:.5f}, {6:.5f}, {7:.5f}, {8:.5f}, {9:.5f};"\
						.format(data_time, mx, my, mz, ax, ay, az, gx, gy, gz))

					Roomba.Move(forward_value, spin_value) # Spin the Roomba toward the desired heading

				# End if
			except KeyboardInterrupt:
				break # Break out of the loop early
			# End try
		# End while distance
		forward_value = 0 # initial forward speed value (mm/s)
		spin_value = 0 # initial spin speed value (mm/s)
		Roomba.Move(forward_value, spin_value) # Stop Roomba movement
		Roomba.PauseQueryStream()
		if Roomba.Available() > 0:
			x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
			#print(x) # Include for debugging purposes
		# End if
		# Return to top of loop; ask for new direction.
	except KeyboardInterrupt:
		print("") # Move cursor down a line
		break # End the loop

## -- Ending Code Starts Here -- ##
Roomba.Move(0,0)
Roomba.PauseQueryStream()
if Roomba.Available() > 0:
	x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
	#print(x) # Include for debugging purposes
# End if
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
