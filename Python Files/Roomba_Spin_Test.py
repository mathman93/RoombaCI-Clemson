''' 
Roomba_Spin_Test.py
Purpose: Testing communication between Roomba and RPi
	Form basis of Roomba code for other tests.
Last Modified: 3/16/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

## Variables and Constants ##
global Roomba # Specifies connection to Roomba
Roomba = serial.Serial('/dev/ttyS0', 115200)
# LED pin numbers
yled = 17
rled = 27
gled = 22
## Roomba DD pin
ddPin = 23

## Functions and Definitions ##

# Blinks the clean button on Roomba during startup
def BlinkCleanLight():
	#Syntax: [139] [LED code] [LED color] [LED Intesnity]
	Roomba.write('\x8b\x19\x00\x80') # Turn on Dirt Detect light and Green Clean button
	time.sleep(0.5)
	Roomba.write('\x8b\x19\xff\x80') # Change green to red
	time.sleep(0.5)
	Roomba.write('\x8b\x19\xff\x00') # Turn off Clean button
	time.sleep(0.05)
	

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
# Display Running Sketch Info

# LED Pin setup
GPIO.setup(yled, GPIO.OUT)
GPIO.setup(rled, GPIO.OUT)
GPIO.setup(gled, GPIO.OUT)
GPIO.setup(ddPin, GPIO.OUT, initial=GPIO.LOW)
 
# Baud rate of Roomba should be 115200.

# Wake Up Roomba Sequence
Roomba.write('\x07') # Restart Roomba
time.sleep(8) # wait 8 seconds before continuing

GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" STARTING ROOMBA... ")
Roomba.write('\x80') # START command
time.sleep(1)
Roomba.write('\x83') # Control command
# 131 = x83 = Safe Mode
# 132 = x84 = Full Mode (Be ready to catch it!)
time.sleep(0.1)

BlinkCleanLight() # Blink the Clean light on Roomba

# Main Code #
x = 0
# Move Roomba
while True:
	Roomba.write('\x91\x00\x64\x00\x64') # move forward
	time.sleep(2)
	Roomba.write('\x91\xff\x9c\x00\x64') # Turn right
	time.sleep(2)
	x = x + 1
	if x == 3:
		break

Roomba.write('\x91\x00\x00\x00\x00') # stop moving
time.sleep(0.1)

# SMB Theme song.
Roomba.write('\x8c\x00\x0b\x4c\x08\x4c\x08\x1d\x08\x4c\x08\x1d\x08\x48\x08\x4c\x08\x1d\x08\x4f\x08\x1d\x18\x43\x08')
time.sleep(0.05)

# Play the song we just programmed.
Roomba.write('\x8d\x00') # Play song #0
time.sleep(2) # wait for the song to complete

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.write('\x80') # Send Roomba to Passive Mode
Roomba.write('\xad') # STOP Roomba OI
time.sleep(0.5)
Roomba.close() # Close the Roomba serial port.
GPIO.cleanup() # Reset GPIO pins for next program
