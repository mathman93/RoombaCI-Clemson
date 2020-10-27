'''
Roomba_WallFinder_Test.py
Purpose: Test code to have Roomba report when it has hit an obstacle
Last Modified: 11/11/19
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math
import random

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##
''' Displays current date and time to screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake up Roomba sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" Starting ROOMBA...")
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
Roomba.ddPin = 23 # Set Roomba dd pin number
GPIO.setup(Roomba.ddPin, GPIO.OUT, initial=GPIO.LOW)
Roomba.WakeUp(131) # Start up Roomba in Safe Mode
# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
Roomba.BlinkCleanLight() # Blink the Clean light on Roomba

if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	#print(x) # for debugging

print(" ROOMBA Setup Complete")

# Open a text file for data retrieval
#file_name_input = input("Name for data file: ")
#dir_path = "/home/pi/RoombaCI-Clemson/Data_Files/2020_Spring/" # Directory path to save file
#file_name = os.path.join(dir_path, file_name_input+".txt") # Add text file extension
#file = open(file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

# if the IMU is used later, put that setup code here

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # for debugging

# initalize speeds
movSpd = 138 # initializes move speed
spnspd = 75

# initialize timers
#spinTime = (235 * math.pi) / (4 * spnspd) # from formula
spinTime = 0.75 # arbitrary number
backTime = 0.25
dataTimer = time.time()
timer = time.time()
moveHelper = (time.time() - (spinTime + backTime))

# initialize values
spinVal = 0 # was 100, just trying something
moveVal = 0
#bumper_byte = 0
last_bumper = 0 # tracks which bumper was hit on previous loop
last_encoder_left = 0
left_difference = 0
last_encoder_right = 0
right_difference = 0
stuck_count = 0
forwardSpin = 0

[bumper_byte,left_start,right_start]=Roomba.Query(7,43,44) # Initial wheel encoder counts

y_position = 0
x_position = 0
theta = 0
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev
data_time = time.time()

# Write initial data to file
#file.write("{0:.3f},{1},{2},{3:.3f},{4:.3f},{5:.5f},{6:0>8b}\n".format(0,left_start,right_start,x_position,y_position,theta,bumper_byte))

# Main Code #
Roomba.Move(0,0) # Start Roomba moving


# New code implementation 
Roomba.StartQueryStream(45)
if Roomba.Available() > 0:
[newcode] = Roomba.ReadQueryStream(45)





Roomba.StartQueryStream(7, 43, 44) # Start query stream with specific sensor packets
# can add other packets later if needed
while True:
	try:
		if (time.time() - timer) > 0.5:
			#flickers green led for checking if it works
			if GPIO.input(gled) == True:
				GPIO.output(gled, 0)
			else:
				GPIO.output(gled, 1)
			timer = time.time()	

		if Roomba.Available() > 0:
			data_time2 = time.time()
			bumper_byte, l_counts, r_counts = Roomba.ReadQueryStream(7, 43, 44)
			print("{0:0>8b}, {1}, {2}".format(bumper_byte, l_counts, r_counts)) #check syntax
			delta_l = l_counts-left_start
			delta_r = r_counts-right_start
			# Determine the change in theta and what that is currently
			delta_theta = (delta_l-delta_r)*C_theta
			theta += delta_theta
			# Determine what method to use to find the change in distance
			if delta_l-delta_r == 0:
				delta_d = 0.5*(delta_l+delta_r)*distance_per_count
			else:
				delta_d = 2*235*((delta_l/(delta_l-delta_r))-.5)*math.sin(delta_theta/2)
			# Find new x and y position
			x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
			y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)
			# write the time, left encoder, right encoder, x position, y position, and theta
			#file.write("{0:.3f},{1},{2},{3:.3f},{4:.3f},{5:.5f},{6:0>8b}\n".format(data_time2-data_time,l_counts,r_counts,x_position,y_position,theta,bumper_byte))
			left_start = l_counts
			right_start = r_counts
			
			# Bumper logic
			if (bumper_byte % 4) > 0:
				moveHelper = time.time()
				if (bumper_byte % 4) == 1:
					# right bump
					print("Right bumper hit!")
					spinVal = -spnspd
					moveVal = -100
					last_bump = 1
				elif (bumper_byte % 4) == 2:
					# left bump
					print("Left bumper hit!")
					spinVal = spnspd
					moveVal = -100
					last_bump = 2
				else: 
					# both - front hit
					print("Both bumpers hit!")
					y = random.randint(0,1)
					spinVal = random.randint(spnspd - 50, spnspd + 50)
					if y == 0:
						spinVal = -spinVal
					moveVal = -100
					last_bump = 3
				forwardSpin = int(-spinVal / 2)

				 Unimplemented
				l_difference = abs(last_encoder_left - l_counts)
				r_difference = abs(last_encoder_right - r_counts)
				#if ((l_difference > 300) AND (r_difference > 300)):
				#	stuck_count += 1
				last_encoder_left = l_counts
				last_encoder_right = r_counts'''
			
			#timer for the backward movement, then the spin
			if (time.time() - moveHelper) < backTime:
				Roomba.Move(moveVal, 0) # backward movement
			elif (time.time() - moveHelper) < (backTime + spinTime):
				Roomba.Move(0, spinVal) # spin
			else: 
				Roomba.Move(movSpd, forwardSpin) # forward and spin

	except KeyboardInterrupt:
		print('')
		break # exit while loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PauseQueryStream() # Pause Query Stream before ending program
Roomba.Move(0,0) # Stop Roomba movement
x = Roomba.DirectRead(Roomba.Available()) # Clear buffer
#file.close() # Close data file
Roomba.PlaySMB()
GPIO.output(gled, GPIO.LOW) # turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program