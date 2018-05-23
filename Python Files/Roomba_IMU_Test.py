'''
Roomba_IMU_Test.py
Purpose: Testing communication between Roomba and LSM9DS1 IMU
	Form basis of Roomba code for other tests.
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 5/23/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import math
#from math import atan2, degrees

## Import other Files ##
import IMU_lib
#from test import *

## Variables and Constants ##
global Roomba # Specifies connection to Roomba
global Xbee # Specifies connection to Xbee
Roomba = serial.Serial('/dev/ttyS0', 115200) # Baud rate should be 115200
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13
## Roomba DD pin
ddPin = 23

## Functions and Definitions ##
''' Converts integers into bytes (Ints To Bytes)
	Only does integers in range [0, 255] '''
def itb(num):
	return (num).to_bytes(1, byteorder='big', signed=False)

''' Blinks the clean button on Roomba during startup
	Helps determine that RPi -> Roomba communication is working'''
def BlinkCleanLight():
	#Syntax: [139] [LED code] [LED color] [LED Intesnity]
	# Turn on Dirt Detect light and Green Clean button
	Roomba.write(itb(139))
	Roomba.write(itb(25))
	Roomba.write(itb(0))
	Roomba.write(itb(128))
	time.sleep(0.5)
	# Change green to red
	Roomba.write(itb(139))
	Roomba.write(itb(25))
	Roomba.write(itb(255))
	Roomba.write(itb(128))
	time.sleep(0.5)
	# Turn off Clean button
	Roomba.write(itb(139))
	Roomba.write(itb(25))
	Roomba.write(itb(255))
	Roomba.write(itb(0))
	time.sleep(0.05)

''' Send command to Roomba to move
	x is common wheel speed (mm/s); y is diffential wheel speed (mm/s)
	x > 0 -> forward motion; y > 0 -> CW motion
	Error may result if |x| + |y| > 500. '''
def Move(x,y):
	RW = x - y # Right wheel speed
	LW = x + y # Left wheel speed
	Roomba.write(itb(145)) # Send command to Roomba to set wheel speeds
	Roomba.write((RW).to_bytes(2, byteorder='big', signed=True))
	Roomba.write((LW).to_bytes(2, byteorder='big', signed=True))

def Play_SMB():
	#Define SMB Theme song
	Roomba.write(itb(140))
	Roomba.write(itb(0))
	Roomba.write(itb(11))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(8))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(8))
	Roomba.write(itb(72))
	Roomba.write(itb(8))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(8))
	Roomba.write(itb(79))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(24))
	Roomba.write(itb(67))
	Roomba.write(itb(8))

	time.sleep(0.05)
	#Play song
	Roomba.write(itb(141))
	Roomba.write(itb(0))
	time.sleep(2) # Wait for song to play

''' Displays current date and time to the screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
# Display Running Sketch Info (?)

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ddPin, GPIO.OUT, initial=GPIO.LOW)

# Wake Up Roomba Sequence
Roomba.write(itb(7)) # Restart Roomba
time.sleep(8) # wait 8 seconds before continuing

GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" STARTING ROOMBA... ")
Roomba.write(itb(128)) # START command
time.sleep(1)
Roomba.write(itb(131)) # Control command
# 131 = Safe Mode
# 132 = Full Mode (Be ready to catch it!)
time.sleep(0.1)

BlinkCleanLight() # Blink the Clean light on Roomba

# Main Code #

# Initialize Magnetometer
mag = IMU_lib.Magnetometer()

Move(0,75)
mag.Calibrate()
Move(0,0)

print("x_min= %d "%(mag.x_min))
print("x_max= %d "%(mag.x_max))
print("y_min= %d "%(mag.y_min))
print("y_max= %d "%(mag.y_max))
print()

basetime = time.time()
basetime_offset = 0.1

while True:
	try:
		if (time.time() - basetime) > basetime_offset:
			[mx,my] = mag.read() # Read magnetometer x and y values
			angle = CalculateHeading() # Calculate heading
			# Note: angle may not correspond to mx, my
			print("%f, %f, %f;"%(angle, mx, my))
		
	except KeyboardInterrupt: # When commanded to stop program (Ctrl + C)...
		print('') # Print new line
		break # Exit while loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Move(0,0)
Play_SMB()
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.write(itb(128)) # Send Roomba to Passive Mode
Roomba.write(itb(174)) # STOP Roomba OI
time.sleep(0.05)
Roomba.close() # Close the Roomba serial port.
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
