''' Roomba_Control_Test.py
Purpose: Use Arrow Keys to control Roomba movement
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/6/2019
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import curses
import math
import RoombaCI_lib # Make sure this file is in the same directory

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

# Clear window and start up "curses"
stdscr = curses.initscr()
def MyLoop(stdscr):
	curses.echo() # Echoes keyboard input to the screen
	#curses.noecho() # Turns off keyboard echo
	
	# Initialize forward and spin values
	forward = 0
	spin = 0
	data_counter = 0
	distance = 0.0 # total distance travelled (millimeters)
	x_pos = 0.0 # initial x-direction position (millimeters)
	y_pos = 0.0 # initial y-direction position (millimeters)
	
	# Get initial angle from IMU
	angle = imu.CalculateHeading()
	
	# Display initial information to the screen
	stdscr.addstr(0,0,"You are now in control of the Roomba.")
	stdscr.addstr(1,0,"Use arrow keys to move; Use Q to stop; Use Ctrl+c to exit.")
	stdscr.addstr(3,0,"Forward: 0")
	stdscr.addstr(4,0,"Spin: 0")
	
	stdscr.nodelay(1) # Don't wait for input when calling getch()
	
	Roomba.SendQuery(7,43,44,42,41,45)
	while Roomba.Available() == 0:
		pass # Wait for sensor packet values to be returned
	bumper_byte, l_counts_current, r_counts_current, l_speed, r_speed, light_bumper = Roomba.ReadQuery(7, 43, 44, 42, 41, 45) # Read new wheel counts

	# Start Query Data Stream
	Roomba.StartQueryStream(7,43,44,42,41,45)
	
	speed_step = 25 # mm/s
	query_timer = (1/64) # seconds
	stop_base = time.time()
	time_base = time.time()
	query_base = time.time()
	while True:
		try:
			# Read query stream for specific packets (ReadQueryStream)
			'''if (time.time() - query_base) > query_timer:
				bumper_byte, l_counts, r_counts, light_bumper, r_speed, l_speed = Roomba.Query(7,43,44,45,41,42)
				current_time = (time.time() - time_base) # Current time of data
				stdscr.move(5,0)
				stdscr.clrtoeol() # Clear line
				stdscr.addstr(5,0,"{0:.4f}, {1:0>8b}, {2}, {3}, {4:0>8b}, {5}, {6};".format(current_time, bumper_byte, l_counts, r_counts, light_bumper, l_speed, r_speed))
				data_counter += 1 # Increment counter for the next data sample
				query_base += query_timer # Update query_base value for next query
			'''
			if Roomba.Available() > 0:
				bumper_byte, l_counts, r_counts, l_speed, r_speed, light_bumper = Roomba.ReadQueryStream(7,43,44,42,41,45) # Read new wheel counts
				
				# Record the current time since the beginning of loop
				current_time = (time.time() - time_base) # Current time of data
				
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
				
				delta_angle = (delta_l_count-delta_r_count)*TURN_CONSTANT
				angle += delta_angle
				if angle >= 360 or angle < 0:
					angle = (angle % 360) # Normalize the angle value from [0,360)
				# Determine what method to use to find the change in distance
				if delta_l_count-delta_r_count == 0: # delta_angle == 0
					delta_distance = 0.5*(delta_l_count+delta_r_count)*DISTANCE_CONSTANT
				else: # delta_angle != 0
					delta_distance = 2*(WHEEL_SEPARATION*(delta_l_count/(delta_l_count-delta_r_count)-.5))*math.sin(math.radians(delta_angle/2))
				distance += delta_distance # Updated distance of Roomba
				# Find new x and y position
				x_pos += delta_distance*math.cos(math.radians(angle-.5*delta_angle))
				y_pos += delta_distance*math.sin(math.radians(angle-.5*delta_angle))
				
				# Measure IMU values
				accel_x, accel_y, accel_z = imu.acceleration
				mag_x, mag_y, mag_z = imu.magnetic
				gyro_x, gyro_y, gyro_z = imu.gyro

				# Display current data to the screen
				stdscr.move(5,0)
				stdscr.clrtoeol() # Clear line
				stdscr.addstr(5,0,"{0:.4f}, {1:0>8b}, {2}, {3}, {4:0>8b}, {5}, {6}, {7:.3f}, {8:.3f}, {9:.2f};".format(current_time, bumper_byte, l_counts, r_counts, light_bumper, l_speed, r_speed, y_pos, x_pos, angle))
				stdscr.move(6,0)
				stdscr.clrtoeol() # Clear line
				stdscr.addstr(6,0,"{0:.3f}, {1:.3f}, {2:.3f}; (9.8 m/s^2)".format(accel_x, accel_y, accel_z))
				stdscr.move(7,0)
				stdscr.clrtoeol() # Clear line
				stdscr.addstr(7,0,"{0:.3f}, {1:.3f}, {2:.3f}; (Gauss)".format(mag_x, mag_y, mag_z))
				stdscr.move(8,0)
				stdscr.clrtoeol() # Clear line
				stdscr.addstr(8,0,"{0:.3f}, {1:.3f}, {2:.3f}; (deg/s)".format(gyro_x, gyro_y, gyro_z))
				
				# Update current wheel encoder counts
				l_counts_current = l_counts
				r_counts_current = r_counts
			
			ctrl_char = stdscr.getch(2,0)
			# Returns -1 if there is no input
			if ctrl_char != -1: # If there is an input
				stop_base = time.time()
				# Reset values
				if ctrl_char == 113: # "q"
					forward = 0
					spin = 0
				# Update values
				elif ctrl_char == curses.KEY_UP: # "w"
					forward += speed_step # Increase forward speed
				elif ctrl_char == curses.KEY_DOWN: # "s"
					forward -= speed_step # Decrease forward speed
				elif ctrl_char == curses.KEY_RIGHT: # "d"
					spin += speed_step # Spin CW
				elif ctrl_char == curses.KEY_LEFT: # "a"
					spin -= speed_step # Spin CCW
				else:
					pass # Do nothing
				
				# Cap off values
				if forward > 250:
					forward = 250
				elif forward < -250:
					forward = -250
				if spin > 150:
					spin = 150
				elif spin < -150:
					spin = -150
			
			# Update forward and spin values to the screen
			stdscr.move(3,0)
			stdscr.clrtoeol() # Clear line
			stdscr.addstr(3,0,"Forward: {}".format(forward))
			stdscr.move(4,0)
			stdscr.clrtoeol() # Clear line
			stdscr.addstr(4,0,"Spin: {}".format(spin))
			# Set Roomba movements to updated values
			Roomba.Move(forward, spin)
			
		except KeyboardInterrupt:
			print('') # print new line
			break # exit while loop
	
	Roomba.Move(0,0) # Stop Roomba movement
	Roomba.PauseQueryStream()
	if Roomba.Available() > 0:
		x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
		#print(x) # Include for debugging purposes

curses.wrapper(MyLoop) # Call "MyLoop" function in the curses wrapper
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
#Roomba.PlaySMB()
GPIO.output(gled, GPIO.LOW) # Turn off green LED

curses.echo()
curses.endwin()
Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
