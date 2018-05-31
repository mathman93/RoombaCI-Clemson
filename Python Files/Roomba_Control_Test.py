''' Roomba_Control_Test.py
Purpose: Use Arrow Keys to control Roomba movement
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 5/31/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import curses

import Roomba_lib # Make sure this file is in the same directory
import IMU_lib # Make sure this file is in the same directory

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 57600
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##
''' Displays current date and time to the screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

''' Takes an integer and find the sum of its byte values'''
def ByteValueSum(number,bytes,sign):
	buffer = (number).to_bytes(bytes, byteorder='big', signed=sign) # Convert to bytes
	num = int.from_bytes(buffer, byteorder='big', signed=False) # Convert back to integer
	total = 0
	while num > 255:
		total += (num % 256) # value of low byte
		num = (num // 256) # value of the rest 
	total += num # add in the last part
	return total

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
Roomba = Roomba_lib.Create_2("/dev/ttyS0", 115200)
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
imu = IMU_lib.LSM9DS1_IMU() # Initialize IMU
time.sleep(0.5)
# Calibrate IMU
print(" Calibrating IMU...")
Roomba.Move(0,75) # Start Roomba spinning
imu.CalibrateMag() # Calculate magnetometer offset values
Roomba.Move(0,0) # Stop Roomba spinning
time.sleep(0.5)
imu.CalibrateAccelGyro() # Calculate accelerometer and gyroscope offset values
# Display offset values
print("mx_offset = {:f}; my_offset = {:f}; mz_offset = {:f}".format(imu.mx_offset, imu.my_offset, imu.mz_offset))
print("ax_offset = {:f}; ay_offset = {:f}; az_offset = {:f}".format(imu.ax_offset, imu.ay_offset, imu.az_offset))
print("gx_offset = {:f}; gy_offset = {:f}; gz_offset = {:f}".format(imu.gx_offset, imu.gy_offset, imu.gz_offset))
print(" IMU Setup Complete")
time.sleep(1) # Gives time to read offset values before continuing
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
	# Request data packets from Roomba (Stream)
	Roomba.StartQueryStream(7, 43, 44, 45, 41, 42) # Start query stream with specific sensor packets
	
	# Initialize forward and spin values
	forward = 0
	spin = 0
	data_counter = 0
	# Display initial information to the screen
	stdscr.addstr(0,0,"You are now in control of the Roomba.")
	stdscr.addstr(1,0,"Use WASD to move; Use Q to stop; Use Ctrl+C to exit.")
	stdscr.addstr(3,0,"Forward: 0")
	stdscr.addstr(4,0,"Spin: 0")
	
	stdscr.nodelay(1) # Don't wait for input when calling getch()
	while True:
		try:
			# Read query stream for specific packets (ReadQueryStream)
			if Roomba.Available() > 0:
				bumper_byte, l_counts, r_counts, light_bumper, r_speed, l_speed = Roomba.ReadQueryStream(7, 43, 44, 45, 41, 42)
				stdscr.addstr(5,0,"                                                            ")
				stdscr.addstr(5,0,"{0}, {1:0>8b}, {2}, {3}, {4:0>8b}, {5}, {6};".format(data_counter, bumper_byte, l_counts, r_counts, light_bumper, l_speed, r_speed))
				data_counter += 1 # Increment counter for the next data sample
			
			ctrl_char = stdscr.getch(2,0)
			# Returns -1 if there is no input
			if ctrl_char != -1: # If there is an input
				# Reset values
				if ctrl_char == 113: # "q"
					forward = 0
					spin = 0
				# Update values
				elif ctrl_char == 119: # "w"
					forward += 25 # Increase forward speed
				elif ctrl_char == 115: # "s"
					forward -= 25 # Decrease forward speed
				elif ctrl_char == 100: # "d"
					spin += 25 # Spin CW
				elif ctrl_char == 97: # "a"
					spin -= 25 # Spin CCW
				else:
					pass # Do nothing
				
				# Cap off values
				if forward > 250:
					forward = 250
				if forward < -250:
					forward = -250
				if spin > 150:
					spin = 150
				if spin < -150:
					spin = -150
				
				# Update forward and spin values to the screen
				stdscr.addstr(3,0,"              ") # Clear line
				stdscr.addstr(3,0,"Forward: {}".format(forward))
				stdscr.addstr(4,0,"           ") # Clear line
				stdscr.addstr(4,0,"Spin: {}".format(spin))
				# Set Roomba movements to updated values
				Roomba.Move(forward, spin)
			
		except KeyboardInterrupt:
			print('') # print new line
			break # exit while loop

curses.wrapper(MyLoop)
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PauseQueryStream()
Roomba.Move(0,0) # Stop Roomba movement
x = Roomba.DirectRead(Roomba.Available()) # Clear buffer
Roomba.PlaySMB()
GPIO.output(gled, GPIO.LOW) # Turn off green LED

curses.echo()
curses.endwin()
Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
