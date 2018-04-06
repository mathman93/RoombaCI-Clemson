''' 
Roomba_DataRead_Test.py
Purpose: Testing communication between Roomba and RPi
	Form basis of Roomba code for other tests.
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 4/6/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

## Variables and Constants ##
global Roomba # Specifies connection to Roomba
global Xbee # Specifies connection to Xbee
Roomba = serial.Serial('/dev/ttyS0', 115200) # Baud rate should be 115200
Xbee = serial.Serial('/dev/ttyUSB0', 57600) # Baud rate should be 57600
# LED pin numbers
yled = 17
rled = 27
gled = 22
## Roomba DD pin
ddPin = 23

## Functions and Definitions ##
# Converts integers into bytes (Ints To Bytes)
# Only does integers in range [0, 255]
def itb(num):
	return (num).to_bytes(1, byteorder='big', signed=False)

# Blinks the clean button on Roomba during startup
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

# Send command to Roomba to move
# x is common wheel speed (mm/s); y is diffential wheel speed (mm/s)
# x > 0 -> forward motion; y > 0 -> CW motion
# Error may result if |x| + |y| > 500.
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

if Roomba.inWaiting() > 0:
	x = Roomba.read(Roomba.inWaiting()) # Clear out Roomba boot-up info
	#print(x) # Include for debugging

# Main Code #
query_time = time.time() # Set base time for query
query_time_offset = 0.1 # Set time offset for query
data_counter = 0 # Initialize data counter

Move(0,-100) # Start Roomba moving
while True:
	try:
		# Request data packets from Roomba
		if (time.time() - query_time) > query_time_offset: # If enough time has passed
			#Roomba.write(itb(142)) # Ask for a single packet (142)
			Roomba.write(itb(149)) # Ask for a query (149)
			Roomba.write(itb(3))   # Ask for # of packets
			Roomba.write(itb(7))   # Bumper byte packet (1 byte)
			Roomba.write(itb(43))  # Right Wheel encoder counts
			Roomba.write(itb(44))  # Left Wheel encoder counts
			query_time += query_time_offset # offset query time for next query
			# May potentially replace this with "Query Stream"
		# Recieve data from Roomba
		if Roomba.inWaiting() > 4: # If packets are received
			bumper_byte = int.from_bytes(Roomba.read(1), byteorder='big') # Read in one byte
			r_wheel = int.from_bytes(Roomba.read(2), byteorder='big') # Read in two bytes
			l_wheel = int.from_bytes(Roomba.read(2), byteorder='big') # Read in two bytes
			# Print data to screen (MATLAB format)
			print(data_counter, bumper_byte, r_wheel, l_wheel, sep=', ', end=';\n')
			data_counter += 1 #Increment counter for the next data sample
		
	except KeyboardInterrupt:
		print('') # print new line
		break # exit while loop

Move(0,0) # Stop Roomba
# SMB Theme song.
Play_SMB()

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.write(itb(128)) # Send Roomba to Passive Mode
Roomba.write(itb(174)) # STOP Roomba OI
time.sleep(0.05)
Roomba.close() # Close the Roomba serial port.
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
