''' 
Roomba_Read_Test.py
Purpose: Testing communication between Roomba and RPi
	Testing communication between Xbee modules on separate RPi
	Form basis of Roomba code for other tests.
Last Modified: 3/30/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

## Variables and Constants ##
global Roomba # Specifies connection to Roomba
global Xbee # Specifies connection to Xbee
Roomba = serial.Serial('/dev/ttyS0', 115200)
Xbee = serial.Serial('/dev/ttyUSB0', 57600)
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
	# Turn on Dirt Detect light and Green Clean button
	Roomba.write(chr(139))
	Roomba.write(chr(25))
	Roomba.write(chr(0))
	Roomba.write(chr(128))
	time.sleep(0.5)
	# Change green to red
	Roomba.write(chr(139))
	Roomba.write(chr(25))
	Roomba.write(chr(255))
	Roomba.write(chr(128))
	time.sleep(0.5)
	# Turn off Clean button
	Roomba.write(chr(139))
	Roomba.write(chr(25))
	Roomba.write(chr(255))
	Roomba.write(chr(0))
	time.sleep(0.05)

def Move_forward():
	Roomba.write(chr(145))
	Roomba.write(chr(0))
	Roomba.write(chr(100))
	Roomba.write(chr(0))
	Roomba.write(chr(100))

def Move_turn_right():
	Roomba.write(chr(145))
	Roomba.write(chr(255))
	Roomba.write(chr(156))
	Roomba.write(chr(0))
	Roomba.write(chr(100))

def Move_stop():
	Roomba.write(chr(145))
	Roomba.write(chr(0))
	Roomba.write(chr(0))
	Roomba.write(chr(0))
	Roomba.write(chr(0))

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
Roomba.write(chr(7)) # Restart Roomba
time.sleep(8) # wait 8 seconds before continuing

GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" STARTING ROOMBA... ")
Roomba.write(chr(128)) # START command
time.sleep(1)
Roomba.write(chr(131)) # Control command
# 131 = Safe Mode
# 132 = Full Mode (Be ready to catch it!)
time.sleep(0.1)

BlinkCleanLight() # Blink the Clean light on Roomba

# Main Code #
message = 0 # variable to stick message
x = 0
while True:
	try:
		if Xbee.inWaiting() > 0: # If there is something in the receive buffer
			message = Xbee.read(Xbee.inWaiting()) # Read all data in
			print(message) # Display message to screen
		#time.sleep(0.2)
	except KeyboardInterrupt:
		print('')
		break

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.write(chr(128)) # Send Roomba to Passive Mode
Roomba.write(chr(174)) # STOP Roomba OI
time.sleep(0.05)
Roomba.close() # Close the Roomba serial port.
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
