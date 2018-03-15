''' 
Roomba_Spin_Test.py
Purpose: Testing communication between Roomba and RPi
	Form basis of Roomba code for other tests.
Last Modified: 3/15/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

## Variables and Constants ##
global Roomba # Specifies connection to Roomba
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
	Roomba.write(bin(139)) # Turn on Dirt Detect light and Green Clean Button
	Roomba.write(bin(25))
	Roomba.write(bin(0))
	Roomba.write(bin(128))
	time.sleep(0.5)
	Roomba.write(bin(139)) # Change Green to Red
	Roomba.write(bin(25))
	Roomba.write(bin(255))
	Roomba.write(bin(128))
	time.sleep(0.5)
	Roomba.write(bin(139)) # Turn off Clean Button
	Roomba.write(bin(25))
	Roomba.write(bin(255))
	Roomba.write(bin(0))
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

# Initialize connection to Roomba
Roomba = serial.Serial('/dev/ttyS0', 115200) 
# Baud rate of Roomba should be 115200.

# Wake Up Roomba Sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive

Roomba.write(bin(7)) # Restart Roomba
time.sleep(10) # wait 10 seconds before continuing

print("\n STARTING ROOMBA")
Roomba.write(bin(128)) # START command
time.sleep(0.05)
Roomba.write(bin(131)) # Control command
# 131 = Safe Mode
# 132 = Full Mode (Be ready to catch it!)
time.sleep(0.1)

BlinkCleanLight() # Blink the Clean light on Roomba

# Main Code #

# Program a five-note start song into Roomba.
#Roomba.write('\x8c\x00\x05C\x10H\x18J\x08L\x10O\x20') # Original code
Roomba.write(bin(140)) # Write a new song
Roomba.write(bin(0))   # Song # to write
Roomba.write(bin(60))  # C
Roomba.write(bin(16))  # note duration
Roomba.write(bin(64))  # E
Roomba.write(bin(16))  # note duration
Roomba.write(bin(67))  # G
Roomba.write(bin(16))  # note duration
Roomba.write(bin(72))  # C
Roomba.write(bin(32))  # note duration
time.sleep(0.05)

# Play the song we just programmed.
#Roomba.write('\x8d\x00')
Roomba.write(bin(141)) # Play a song
Roomba.write(bin(0))   # Song # to play
time.sleep(2) # wait for the song to complete

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.write(bin(173)) # STOP command
time.sleep(0.05)
Roomba.close() # Close the Roomba serial port.
GPIO.cleanup() # Reset GPIO pins for next program
