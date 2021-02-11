''' WheelEncoderTestCode.py
Purpose: Read wheel encoder values and calculate trajectory
Last Modified: 2/6/2021
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math

## Variables and Constants ##
file_create = False # Boolean to set for creation of data file

## Functions and Definitions ##

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
RoombaCI_lib.DisplayDateTime() # Display current date and time
# LED pin numbers
yled = 5
rled = 6
gled = 13
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
# End if Roomba.Available()
# Indicate completion of Roomba setup
print(" ROOMBA Setup Complete")
GPIO.output(gled, GPIO.LOW)

# Main Code #
if file_create == True:	# Open a text file for data retrieval
	file_name_input = input("Name for data file: ") # Ask user for desired file name
	dir_path = "/home/pi/RoombaCI-Clemson/Data_Files/2021_Spring/" # Directory path to save file
	file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
	file = open(file_name, "w") # Open a text file for storing data
		# Will overwrite anything that was in the text file previously
# End if file_create
# Dictionary of move commands
move_dict = {0:[100,0,10], # Move forward
			1:[0,0,2], # Stop
			2:[0,60,7], # Spin ~180 degrees
			3:[0,0,2], # Stop
			4:[100,0,10], # Move forward
			5:[0,0,2], # Stop
			6:[0,-60,7], # Spin ~-180 degrees
			7:[0,0,2], # Stop
			8:[75,50,15], # CW Circular Arc
			9:[75,-50,15], # CCW Circular Arc
			10:[0,0,2], # Stop
			11:[100,0,10], # Move forward
			12:[0,0,2], # Stop
			13:[0,60,7], # Spin ~180 degrees
			14:[0,0,2], # Stop
			15:[100,0,10], # Move forward
			16:[0,0,2], # Stop
			17:[0,-60,7], # Spin ~-180 degrees
			18:[75,50,15], # CW Circular Arc
			19:[75,-50,15], # CCW Circular Arc
			20:[75,50,15], # CW Circular Arc
			21:[75,-50,15], # CCW Circular ARc
			22:[100,75,45] # CW Circular Arc (to increase wheel count beyond threshold)
			}

# Retrieve and set initial wheel encoder values
[left_encoder, right_encoder] = Roomba.Query(43,44)
Roomba.SetWheelEncoderCounts(left_encoder, right_encoder)

data_start = time.time() # Set time for data reference
if file_create == True:
	file.write("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f}\n"\
		.format(0,Roomba.l_count_last,Roomba.r_count_last,Roomba.X_position,Roomba.Y_position,Roomba.heading))
# End if file_create
Roomba.StartQueryStream(43,44) # Start Roomba Query Stream with wheel encoder data

for i in range(len(move_dict.keys())):
	start_time = time.time() # Set timer for next movement
	# Get peices of dictionary and tell the Roomba to move
	[f,s,t] = move_dict[i]
	Roomba.Move(f,s)
	while time.time() - start_time <= t: # Wait for movement time to finish
		if Roomba.Available() > 0: # If data is available...
			data_time = time.time()-data_start # Get time that the data was retrieved
			[left_encoder, right_encoder] = Roomba.ReadQueryStream(43,44) # Read in left and right wheel encoder values
			Roomba.UpdatePosition(left_encoder, right_encoder) # Update Roomba Position variables
			# Print and write the time, left encoder, right encoder, x position, y position, and heading
			print("Time: {0:.6f}\nLeft Encoder: {1}; Right Encoder: {2}\nX Position: {3:.3f} mm; Y Position: {4:.3f} mm\nHeading (radians): {5:.6f}; Heading (degrees): {6:.3f}"\
				.format(data_time,Roomba.l_count_last,Roomba.r_count_last,Roomba.X_position,Roomba.Y_position,Roomba.heading,Roomba.heading*(180/math.pi)))
			print("")
			if file_create == True:
				file.write("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f}\n"\
					.format(data_time,Roomba.l_count_last,Roomba.r_count_last,Roomba.X_position,Roomba.Y_position,Roomba.heading))
		# End if Roomba.Available()
	# End while
# End for
Roomba.Move(0,0) # Stop moving
Roomba.PauseQueryStream() # End Roomba Query Stream
if Roomba.Available()>0: # If data exists in Query Stream...
	z = Roomba.DirectRead(Roomba.Available()) # Clear out data
	print(z) # Include for debugging
# End if Roomba.Available()
if file_create == True:
	file.close() # Close data file
# End if file_create
Roomba.PlaySMB() # Indicate proper shutdown
## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
