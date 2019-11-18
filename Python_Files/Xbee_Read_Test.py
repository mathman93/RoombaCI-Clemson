''' Xbee_Read_Test.py
Purpose: Testing communication between Xbee modules on separate RPi
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 5/31/2018
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
sendtime_offset = 1.0
basetime = time.time()
basetime_offset = 0.5

if Xbee.inWaiting() > 0:
	junk = XBee.read(Xbee.inWaiting().decode())
	print(junk)
while True:
	try:
		if (time.time() - sendtime) > sendtime_offset:
			message1 = 779.456821 # Make this the number  you want to send
			message2 = -17
			var = "{0:09.3f} {1:09.3f}".format(message1,message2) # Make the string representation of the number
			Xbee.write(var.encode()) # Send the number over the Xbee
			sendtime += sendtime_offset # Increase offset for next message
		
		
		if Xbee.inWaiting() > 18: # If there is something in the receive buffer
			message = Xbee.read(Xbee.inWaiting()).decode() # Read all data in
			print(message) # To see what the string representation is
			coordinate = message.split() # To split the string into x and y coordinates
			print(coordinate) #print the newly made list from the split
			print(coordinate[0])
			#absissa = float(coordinate[0])
			#print(absissa)			
			#print(float(message)) # Display message to screen as a number
		
		if (time.time() - basetime) > basetime_offset: # If enough time has passed.
			if GPIO.input(gled) == True:  # If the LED is on...
				GPIO.output(gled, GPIO.LOW)  # turn it off.
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
