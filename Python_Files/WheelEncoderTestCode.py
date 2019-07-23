''' WheelEncoderTestCode.py
Purpose: Read wheel encoder values and calculate trajecotry
Last Modified: 6/12/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math

## Variables and Constants ##
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
GPIO.output(gled, GPIO.LOW)
# Main Code #
# Open a text file for data retrieval
file_name_input = input("Name for data file: ")
dir_path = "/home/pi/RoombaCI-Clemson/Data_Files/2019_Summer/" # Directory path to save file
file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
file = open(file_name, "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously

start_time = time.time()

# Dictionary of move commands
dict = {0:[100,0,30]}
[left_start,right_start]=Roomba.Query(43,44)

# Variables and Constants
y_position = 0
x_position = 0
theta = 0
wheel_diameter = 72
counts_per_rev = 508.8
distance_between_wheels = 235
C_theta = (wheel_diameter*math.pi)/(counts_per_rev*distance_between_wheels)
distance_per_count = (wheel_diameter*math.pi)/counts_per_rev
data_time = time.time()

file.write("{0},{1},{2},{3},{4},{5}\n".format(0,left_start, right_start,x_position,y_position,theta))
Roomba.StartQueryStream(43,44)


for i in range(len(dict.keys())):
	# Get peices of dictionary and tell the roomba to move
	[f,s,t] = dict[i]
	Roomba.Move(f,s)
	while time.time() - start_time <=t:
		# If data is available
		if Roomba.Available()>0:
			data_time2 = time.time()
			# Get left and right encoder values and find the change in each
			[left_encoder, right_encoder]=Roomba.ReadQueryStream(43,44)
			delta_l = left_encoder-left_start
			delta_r = right_encoder-right_start
			# Determine the change in theta and what that is currently
			delta_theta = (delta_l-delta_r)*C_theta
			theta += delta_theta
			# Determine what method to use to find the change in distance
			if delta_l-delta_r == 0:
				delta_d = 0.5*(delta_l+delta_r)*distance_per_count
			else:
				delta_d = 2*(235*(delta_l/(delta_l-delta_r)-.5))*math.sin(delta_theta/2)
			# Find new x and y position
			x_position = x_position + delta_d*math.cos(theta-.5*delta_theta)
			y_position = y_position + delta_d*math.sin(theta-.5*delta_theta)
			# Print and write the time, left encoder, right encoder, x position, y position, and theta
			print("Time: {0}\nLeft Encoder:{1} Right Encoder: {2}\nX Position:{3} Y Position:{4} Theta:{5}".format(data_time2-data_time,left_encoder,right_encoder,x_position,y_position,theta))
			print("")
			file.write("{0},{1},{2},{3},{4},{5}\n".format(data_time2-data_time,left_encoder, right_encoder,x_position,y_position,theta))
			left_start = left_encoder
			right_start = right_encoder
	start_time = time.time()
Roomba.Move(0,0)
Roomba.PauseQueryStream()
if Roomba.Available()>0:
	z = Roomba.DirectRead(Roomba.Available())
	print(z)
file.close()
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
