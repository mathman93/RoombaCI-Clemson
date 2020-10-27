'''

'''

import serial
import time
import RPi.GPIO as GPIO
import sys

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
if Xbee.inWaiting() > 0:
	# Clear out Xbee buffer
	x = Xbee.read(Xbee.inWaiting()).decode()
	print(x)

sendtime = time.time()
sendtime_offset = 1.0
basetime = time.time()
basetime_offset = 0.5
roombaname = input('enter roombaname: ')
Dict = {roombaname: [0,0]}
#for line in sys.stdin:
while True:
	try:
		variable=input('input letter: ')
		if variable == 's':
			message1 = input('enter x value: ') # Make this the number  you want to send
			message2 = input('enter y value: ')
			Dict[roombaname] = [message1,message2]
			#var = "{0} {1:09.3f} {2:09.3f}".format(roombaname, message1,message2) # Make the string representation of the number
			var = "{} {} {}".format(roombaname,message1,message2)
			Xbee.write(var.encode()) # Send the number over the Xbee
			sendtime += sendtime_offset # Increase offset for next message
		elif variable == 'p':
			print(Dict)


		if Xbee.inWaiting() > 0: # If there is something in the receive buffer
			message = Xbee.read(Xbee.inWaiting()).decode() # Read all data in
			print(message) # To see what the string representation is
			coordinate = message.split() # To split the string into x and y coordinates
			Dict[coordinate[0]] = [coordinate[1],coordinate[2]]



			#if (time.time() - basetime) > basetime_offset: # If enough time has passed.
			#	if GPIO.input(gled) == True:  # If the LED is on...
			#		GPIO.output(gled, GPIO.LOW)  # turn it off
			#	else:
			#		GPIO.output(gled, GPIO.HIGH) # otherwise, turn it on.
			#	basetime += basetime_offset  # set the next base time
	except KeyboardInterrupt:
		print('The program has ended')
		break

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
