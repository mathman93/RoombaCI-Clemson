''' Roomba_Encoder_Test5.py
Purpose: Uses Roomba wheel encoders to determine angle and distance
	Moves to user input x and y coordinates
	Uses only one "while" loop and uses bump sensors to avoid obstacles
	Writes all data into a text file for data retrieval
	Uses Roomba Query Stream to retrieve data
	Use more accurate coordinate position calculations
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/6/2019
'''
## Import libraries ##
import serial
import time
import math
import random
import RPi.GPIO as GPIO

import RoombaCI_lib
# Import these functions separately for easy use
from RoombaCI_lib import DHTurn
from RoombaCI_lib import DDSpeed

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

# Roomba Constants
WHEEL_DIAMETER = 72 # millimeters
WHEEL_SEPARATION = 235 # millimeters
WHEEL_COUNTS = 508.8 # counts per revolution
DISTANCE_CONSTANT = (WHEEL_DIAMETER * math.pi)/(WHEEL_COUNTS) # millimeters/count
TURN_CONSTANT = (WHEEL_DIAMETER * 180)/(WHEEL_COUNTS * WHEEL_SEPARATION) # degrees/count

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
print("Hello, and welcome to Roomba_Encoder_Test5.")
print("In this program, you get to specify the x,y coordinates you want the Roomba to move to.")
print("The Roomba navigates around any obstacles in its path.")
#print("Any data generated in the run will be stored in a text file.")
'''# Create text file for data storage
data_name = input("File name for data? ")
data_name_string = data_name + ".txt" # Add the ".txt" to the end
datafile = open(data_name_string, "w") # Open a text file for storing data
# Write data header for the data file; includes name of each column
datafile.write("Time Stamp, Total Distance Traveled, Distance to Desired Point, Current Angle, Desired Heading, Left Wheel Encoders, Right Wheel Encoders, Left Wheel Speed, Right Wheel Speed, Y-Position, X-position, Bumper Byte, Light Bumper Byte\n")
'''
# Get initial angle from IMU
#angle = imu.CalculateHeading()
angle = 0

#input the speed
spnspd = 100
speed_step = 20

#times, spin time is from formula
spinTime = (WHEEL_SEPARATION * math.pi) / (4 * spnspd)
backTime = 0.5
#initializes timers
moveHelper = (time.time() - (spinTime + backTime))

# Initial conditions
distance = 0.0 # total distance traveled (millimeters)
x_pos = 0.0 # initial x-direction position (millimeters)
y_pos = 0.0 # initial y-direction position (millimeters)
forward_value = 0 # initial forward speed value (mm/s)
spin_value = 0 # initial spin speed value (mm/s)

# Read in initial wheel count values from Roomba
Roomba.SendQuery(7,43,44,42,41,45)
while Roomba.Available() == 0:
	pass # Wait for sensor packet values to be returned
bumper_byte, l_counts_current, r_counts_current, l_speed, r_speed, light_bumper = Roomba.ReadQuery(7, 43, 44, 42, 41, 45) # Read new wheel counts

while True:
	try:
		# Print current angle of Roomba
		print("Current Location: ({0:.3f}, {1:.3f})".format(x_pos, y_pos))
		# Request for the desired angle to turn to
		x_new = float(input("Desired x-coordinate? "))
		y_new = float(input("Desired y-coordinate? "))
		desired_heading = (math.degrees(math.atan2((y_new - y_pos),(x_new - x_pos))) % 360)
		desired_distance = math.sqrt(pow((y_new - y_pos),2) + pow((x_new - x_pos),2))
		
		data_time = 0.0 # 0 seconds initial
		# Print out initial data point
		#datafile.write("{0:.5f}, {1:.3f}, {2:.3f}, {3:.3f}, {4:.3f}, {5}, {6}, {7}, {8}, {9:.3f}, {10:.3f}, {11:0>8b}, {12:0>8b}\n".format(data_time, distance, desired_distance, angle, desired_heading, l_counts_current, r_counts_current, l_speed, r_speed, y_pos, x_pos, bumper_byte, light_bumper))
		# Start Query Data Stream
		Roomba.StartQueryStream(7,43,44,42,41,45)
		# Restart base timers
		base = time.time()
		
		while desired_distance > 2: # Until we have reached the location...
			try:
				if Roomba.Available() > 0:
					bumper_byte, l_counts, r_counts, l_speed, r_speed, light_bumper = Roomba.ReadQueryStream(7,43,44,42,41,45) # Read new wheel counts
					
					# Record the current time since the beginning of loop
					data_time = time.time() - base
					
					# Calculate the count differences and correct for overflow
					delta_l_count = (l_counts - l_counts_current)
					if delta_l_count > pow(2,15): # 2^15 is somewhat arbitrary
						delta_l_count -= pow(2,16)
					if delta_l_count < -pow(2,15): # 2^15 is somewhat arbitrary
						delta_l_count += pow(2,16)
					delta_r_count = (r_counts - r_counts_current)
					if delta_r_count > pow(2,15): # 2^15 is somewhat arbitrary
						delta_r_count -= pow(2,16)
					if delta_r_count < -pow(2,15): # 2^15 is somewhat arbitrary
						delta_r_count += pow(2,16)
					
					# Calculate the turn angle change since the last counts
					angle_change = TURN_CONSTANT * (delta_l_count - delta_r_count) # degrees
					# Update angle of Roomba and correct for overflow
					angle += angle_change # degrees
					if angle >= 360 or angle < 0:
						angle = (angle % 360) # Normalize the angle value from [0,360)
					
					# Calculate the distance change since the last counts
					if delta_l_count == delta_r_count: # or if angle_change == 0
						# Straight Line distance
						distance_change = 0.5 * DISTANCE_CONSTANT * (delta_l_count + delta_r_count) # millimeters
						# Total distance traveled
						distance += distance_change # millimeters
					else: # Circular Arc distance
						distance_radius = WHEEL_SEPARATION * ((delta_l_count/(delta_l_count - delta_r_count)) - 0.5) # millimeters
						distance_change = 2 * distance_radius * math.sin(0.5 * math.radians(angle_change)) # millimeters
						# Total distance traveled
						distance += (distance_radius * math.radians(angle_change)) # millimeters; Slightly larger than distance_change
					
					# Calculate position data
					delta_x_pos = distance_change * math.cos(math.radians(angle - (0.5 * angle_change)))
					delta_y_pos = distance_change * math.sin(math.radians(angle - (0.5 * angle_change)))
					x_pos += delta_x_pos
					y_pos += delta_y_pos
					
					# The direction from the current position to the desired position
					desired_heading = (math.degrees(math.atan2((y_new - y_pos),(x_new - x_pos))) % 360)
					# The distance from the current position to the desired position
					desired_distance = math.sqrt(pow((y_new - y_pos),2) + pow((x_new - x_pos),2))
					
					# If the bumper is pressed...
					if (bumper_byte % 4) > 0:
						moveHelper = time.time() # Reset the timer
						# Determine spin speed for right bumper press
						if (bumper_byte % 4) == 1:
							spin = -spnspd # Spin left
						# Determine spin speed for left bumper press
						elif (bumper_byte % 4) == 2:
							spin = spnspd # Spin right
						# Deteremine spin speed for both bumpers pressed
						# Uses random int to help get out of corners
						else: # (bumper_byte % 4) == 3
							flip = random.randint(0, 1)
							spin = random.randint(spnspd - 50, spnspd + 50)
							if flip == 0:
								spin *= (-1) # Make it negative 50% of the time
						
					# Timer for the backward movement, then the spin
					if (time.time() - moveHelper) < backTime: # Backward movement stuff
						set_forward_value = -100 
						set_spin_value = 0
					elif (time.time() - moveHelper) < (backTime + spinTime): # Spin movement stuff
						set_forward_value = 0
						set_spin_value = spin # Determined above when bumped
					else: # Normal movement stuff
						set_spin_value = DHTurn(angle, desired_heading, epsilon) # Determine the spin speed to turn toward the desired heading
						set_forward_value = DDSpeed(angle, desired_heading, desired_distance)
						
						# If the light sensors detect something, then slow down
						if light_bumper > 0:
							# Cut the speed in half
							set_forward_value = set_forward_value // 2 
							set_spin_value = set_spin_value // 2
					
					# Increment forward movement speed toward the set speed
					if forward_value < set_forward_value: # If it is less than the set speed...
						forward_value += speed_step # Increment the speed by a step
						if forward_value > set_forward_value: # If the speed went to far...
							forward_value = set_forward_value # Set it to the set speed
					if forward_value > set_forward_value: # If it is more than the set speed...
						forward_value -= speed_step # Decrement the speed by a step
						if forward_value < set_forward_value: # If the speed went to far...
							forward_value = set_forward_value # Set it to the set speed
					# Increment spin movement speed toward the set speed
					if spin_value < set_spin_value: # If it is less than the set speed...
						spin_value += speed_step # Increment the speed by a step
						if spin_value > set_spin_value: # If the speed went to far...
							spin_value = set_spin_value # Set it to the set speed
					if spin_value > set_spin_value: # If it is more than the set speed...
						spin_value -= speed_step # Decrement the speed by a step
						if spin_value < set_spin_value: # If the speed went to far...
							spin_value = set_spin_value # Set it to the set speed
					
					# Print out pertinent data values
					print("{0:.5f}, {1:.3f}, {2:.3f}, {3:.3f}, {4:.3f}, {5:0>8b}, {6:0>8b}, {7}, {8};".format(data_time, desired_distance, angle, y_pos, x_pos, bumper_byte, light_bumper, l_counts, r_counts))
					# Write data values to a text file
					#datafile.write("{0:.5f}, {1:.3f}, {2:.3f}, {3:.3f}, {4:.3f}, {5}, {6}, {7}, {8}, {9:.3f}, {10:.3f}, {11:0>8b}, {12:0>8b}\n".format(data_time, distance, desired_distance, angle, desired_heading, l_counts, r_counts, l_speed, r_speed, y_pos, x_pos, bumper_byte, light_bumper))
					
					Roomba.Move(forward_value, spin_value) # Spin the Roomba toward the desired heading
					
					# Update current wheel encoder counts
					l_counts_current = l_counts
					r_counts_current = r_counts
				# End if Roomba.Available() > 0:
			# End try:
			except KeyboardInterrupt: 
				break # Break out of the loop early when prompted
		# End while desired_distance > 2:
		
		forward_value = 0 # initial forward speed value (mm/s)
		spin_value = 0 # initial spin speed value (mm/s)
		Roomba.Move(forward_value, spin_value) # Stop Roomba movement
		Roomba.PauseQueryStream()
		if Roomba.Available() > 0:
			x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
			#print(x) # Include for debugging purposes
		
	except KeyboardInterrupt:
		print("") # Move cursor down a line
		break # End the program loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PlaySMB() # Include for fun
GPIO.output(gled, GPIO.LOW) # Turn off green LED
#datafile.close()

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close() # Close Xbee serial connection
GPIO.cleanup() # Reset GPIO pins for next program
