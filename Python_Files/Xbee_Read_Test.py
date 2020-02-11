''' Xbee_Read_Test.py
Purpose: Testing communication between Xbee modules on separate RPi
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 2/11/2020
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
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
DisplayDateTime()

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Main Code #
sendtime = time.time()
sendtime_offset = 1.0 # Time between sending messages
basetime = time.time()
basetime_offset = 0.5 # Time between LED blinks

while True:
	try:
		if (time.time() - sendtime) > sendtime_offset:
			message = '1' # Change this to any character string you want
			Xbee.write(message.encode()) # Send the number over the Xbee
			sendtime += sendtime_offset # Increase offset for next time to send message
		
		if Xbee.inWaiting() > 0: # If there is something in the receive buffer
			message = Xbee.read(Xbee.inWaiting()).decode() # Read all data in
			print(message) # To see what the message is
		
		# LED blink conditional, to make sure code is running
		if (time.time() - basetime) > basetime_offset: # If enough time has passed.
			if GPIO.input(gled) == True:  # If the LED is on...
				GPIO.output(gled, GPIO.LOW)  # turn it off
			else:
				GPIO.output(gled, GPIO.HIGH) # otherwise, turn it on.
			basetime += basetime_offset  # set the next base time
		
	except KeyboardInterrupt:
		print('')
		break

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
