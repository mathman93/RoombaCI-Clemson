''' Roomba_Encoder_Test5.py
Purpose: Uses Roomba wheel encoders to determine angle and distance
	Moves to user input x and y coordinates
	Uses only one "while" loop and uses bump sensors to avoid obstacles
	Writes all data into a text file for data retrieval
	Uses Roomba Query Stream to retrieve data
	Use more accurate coordinate position calculations
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 9/19/2018
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


while True:
	try:
		x_new = float(input("Again? "))
		Roomba.Move(0,100)
		time.sleep(50)
		Roomba.Move(0,0)
		
	except KeyboardInterrupt: 
		break # Break out of the loop early if something wrong happens

Roomba.Move(0,0)



## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PlaySMB() # Include for fun
GPIO.output(gled, GPIO.LOW) # Turn off green LED
#datafile.close()

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close() # Close Xbee serial connection
GPIO.cleanup() # Reset GPIO pins for next program
