''' Roomba_Song_Test.py
Purpose: Play user song on Roomba
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/28/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

import RoombaCI_lib

## Variables and Constants ##

# LED pin numbers
gled = 13

## Functions and Definitions ##


## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO

# LED Pin setup
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


# Program the song onto the Roomba

# Zelda Theme (10 notes)
Roomba.DirectWrite(140) # Header Opcode
Roomba.DirectWrite(0)   # Song number (0-3)
Roomba.DirectWrite(10)  # Length of song (0-15)
Roomba.DirectWrite(72)  # Note 1
Roomba.DirectWrite(24)	# Note 1 duration (in 1/64 of a second)
Roomba.DirectWrite(30)
Roomba.DirectWrite(8)
Roomba.DirectWrite(67)
Roomba.DirectWrite(48)
Roomba.DirectWrite(30)
Roomba.DirectWrite(8)
Roomba.DirectWrite(72)
Roomba.DirectWrite(8)
Roomba.DirectWrite(72)
Roomba.DirectWrite(8)
Roomba.DirectWrite(74)
Roomba.DirectWrite(8)
Roomba.DirectWrite(76)
Roomba.DirectWrite(8)
Roomba.DirectWrite(77)
Roomba.DirectWrite(8)
Roomba.DirectWrite(79)
Roomba.DirectWrite(64)

time.sleep(0.05) # Wait

# SuperStar Theme (16 notes)
Roomba.DirectWrite(140) # Header Opcode
Roomba.DirectWrite(1)   # Song number (0-3)
Roomba.DirectWrite(16)   # Length of song (0-15)
Roomba.DirectWrite(77)  # Note 1
Roomba.DirectWrite(12)	# Note 1 duration (in 1/64 of a second)
Roomba.DirectWrite(77)
Roomba.DirectWrite(12)
Roomba.DirectWrite(77)
Roomba.DirectWrite(18)
Roomba.DirectWrite(77)
Roomba.DirectWrite(12)
Roomba.DirectWrite(77)
Roomba.DirectWrite(12)
Roomba.DirectWrite(77)
Roomba.DirectWrite(6)
Roomba.DirectWrite(77)
Roomba.DirectWrite(12)
Roomba.DirectWrite(77)
Roomba.DirectWrite(12)
Roomba.DirectWrite(76)
Roomba.DirectWrite(12)
Roomba.DirectWrite(76)
Roomba.DirectWrite(12)
Roomba.DirectWrite(76)
Roomba.DirectWrite(18)
Roomba.DirectWrite(76)
Roomba.DirectWrite(12)
Roomba.DirectWrite(76)
Roomba.DirectWrite(12)
Roomba.DirectWrite(76)
Roomba.DirectWrite(6)
Roomba.DirectWrite(76)
Roomba.DirectWrite(12)
Roomba.DirectWrite(76)
Roomba.DirectWrite(12)

time.sleep(0.05) # Wait

# SMG Theme (13 notes)
Roomba.DirectWrite(140) # Header Opcode
Roomba.DirectWrite(2)   # Song number (0-3)
Roomba.DirectWrite(13)  # Length of song (0-15)
Roomba.DirectWrite(79)
Roomba.DirectWrite(20)
Roomba.DirectWrite(80)
Roomba.DirectWrite(4)
Roomba.DirectWrite(81)
Roomba.DirectWrite(4)
Roomba.DirectWrite(82)
Roomba.DirectWrite(4)
Roomba.DirectWrite(83)
Roomba.DirectWrite(4)
Roomba.DirectWrite(84)
Roomba.DirectWrite(24)
Roomba.DirectWrite(30)
Roomba.DirectWrite(8)
Roomba.DirectWrite(67)
Roomba.DirectWrite(32)
Roomba.DirectWrite(72)
Roomba.DirectWrite(32)
Roomba.DirectWrite(79)
Roomba.DirectWrite(32)
Roomba.DirectWrite(78)
Roomba.DirectWrite(16)
Roomba.DirectWrite(74)
Roomba.DirectWrite(16)
Roomba.DirectWrite(76)
Roomba.DirectWrite(64)

time.sleep(0.05) # Wait

# Play Zelda
Roomba.DirectWrite(141) # Header Opcode
Roomba.DirectWrite(0)   # Song number (0-3)
time.sleep(4) # Wait for the song to play

while True:
	try:
		# Play SuperStar
		Roomba.DirectWrite(141) # Header Opcode
		Roomba.DirectWrite(1)   # Song number (0-3)
		
		time.sleep(3.02) # Wait for the song to play
		
	except KeyboardInterrupt:
		time.sleep(3)
		# Play SMG
		Roomba.DirectWrite(141) # Header Opcode
		Roomba.DirectWrite(2)   # Song number (0-3)
		time.sleep(4) # Wait for the song to playbreak
		break

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program