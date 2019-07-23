''' Dock_Test.py
Purpose: Python code for automated Roomba docking to charging station
Last Modified: 6/28/2019
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
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

''' Blinks an LED on/off
	'''
def BlinkLED(led, led_bool):
	led_bool = not led_bool
	if led_bool:
		GPIO.output(led, GPIO.HIGH)
	else:
		GPIO.output(led, GPIO.LOW)
	return led_bool

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake Up Roomba Sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive and setting up
print(" Starting ROOMBA...")
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
Roomba.ddPin = 23 # Set Roomba dd pin number
GPIO.setup(Roomba.ddPin, GPIO.OUT, initial=GPIO.HIGH)
# Pulse Device Detection pin (ddPin) to wake Roomba from sleep
# Only seems to work when the Roomba is not docked
GPIO.output(Roomba.ddPin, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(Roomba.ddPin, GPIO.LOW)
time.sleep(0.4)
GPIO.output(Roomba.ddPin, GPIO.HIGH)
time.sleep(0.5)

print("Start OI")
Roomba.DirectWrite(128) # From off, start Roomba OI (sets to Passive)
time.sleep(0.1)
print("Start Safe Mode")
Roomba.DirectWrite(131) # From Passive mode, send to Safe Mode
time.sleep(0.1)
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	print(x) # Include for debugging

Roomba.BlinkCleanLight() # Test if Roomba is in Safe Mode
#StartUp(Roomba, 23, 131) # Start up Roomba in Safe mode
print(" ROOMBA Setup Complete")

GPIO.output(gled, GPIO.LOW) # Turn off green LED to say we have finished setup sequence

# Main Code #
print(" Now Undocking...")
backup_base = time.time()
blink_base = backup_base
yled_bool = False

print("Backing Up")
Roomba.Move(-40,0)
while time.time() - backup_base < 5:
	if time.time() - blink_base > 0.5:
		yled_bool = BlinkLED(yled, yled_bool) # Blink light while backing up
		blink_base += 0.5
	# End if
# End while

print("Stopping")
Roomba.Move(0,0)
GPIO.output(yled, GPIO.LOW)
time.sleep(0.5)

Roomba.PlaySMB() # For fun :)
print(" Now Docking...")
charging_state = 0

Roomba.Dock() # Send Roomba to dock (built-in function)

blink_base = time.time()
Roomba.StartQueryStream(34,35)
while charging_state == 0: # Until Roomba is on the dock
	try:
		if Roomba.Available() > 0:
			[charging_state, oi_state] = Roomba.ReadQueryStream(34,35)
			print("Charging State Value: {0}".format(charging_state))
			print("OI State Value: {0}".format(oi_state))
		if time.time() - blink_base > 0.5:
			print("Blinking #2")
			yled_bool = BlinkLED(yled, yled_bool)
			blink_base += 0.5
	except KeyboardInterrupt:
		break
# End while
print("Reached Dock")
Roomba.PauseQueryStream()
if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	print(x) # Include for debugging
time.sleep(0.1)

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
