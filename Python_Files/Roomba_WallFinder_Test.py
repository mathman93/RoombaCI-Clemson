'''
Roomba_WallFinder_Test.py
Purpose: Test code to have Roomba report when it has hit an obstacle
Last Modified: 11/04/19
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import math
import random

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##
''' Displays current date and time to screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake up Roomba sequence
GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
print(" Starting ROOMBA...")
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)
Roomba.ddPin = 23 # Set Roomba dd pin number
GPIO.setup(Roomba.ddPin, GPIO.OUT, initial=GPIO.LOW)
Roomba.WakeUp(131) # Start up Roomba in Safe Mode
# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
Roomba.BlinkCleanLight() # Blink the Clean light on Roomba

if Roomba.Available() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	#print(x) # for debugging

print(" ROOMBA Setup Complete")

# if the IMU is used later, put that setup code here

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # for debugging

# initalize speeds
movSpd = 138 # initializes move speed
spnspd = 100

# initialize timers
spinTime = (235 * math.pi) / (4 * spnspd) # from formula
backTime = 0.5
dataTimer = time.time()
timer = time.time()
moveHelper = (time.time() - (spinTime + backTime))
# initialize values
spinVal = 100
moveVal = 0
bumper_byte = 0

# Main Code #
query_time = time.time() # set base time for query
query_time_offset = 5*(0.015) # set time offset for query
# smallest time offset for query is 15 ms

Roomba.Move(0,0) # Start Roomba moving

Roomba.StartQueryStream(7, 43, 44) # Start query stream with specific sensor packets
# can add other packets later if needed
while True:
	if (time.time() - timer) > 0.5:
		#flickers green led for checking if it works
		if GPIO.input(gled) == True:
			GPIO.output(gled, 0)
		else:
			GPIO.output(gled, 1)
		timer = time.time()	

	if Roomba.Available() > 0:
		bumper_byte, l_counts, r_counts = Roomba.ReadQueryStream(7, 43, 44)
		print("{0:0>8b}, {1}, {2}".format(bumper_byte, l_counts, r_counts) #check syntax

		# Bumper logic
		if (bumper_byte % 4) > 0:	
			moveHelper = time.time()
			if (bumper_byte % 4) == 1:
				# right bump
				spinVal = -spnspd
				moveVal = -100
			elif (bumper_byte % 4) == 2:
				# left bump
				spinVal = spnspd
				moveVal = -100
			else: 
				# both - front hit
				y = random.randint(0,1)
				spinVal = random.randint(spnspd - 50, spnspd + 50)
				if y == 0:
					spinVal = -spinVal
				moveVal = -100
	
	#timer for the backward movement, then the spin
	if (time.time() - moveHelper) < backTime:
		Roomba.Move(moveVal, 0) # backward movement
	elif (time.time() - moveHelper) < (backTime + spinTime):
		Roomba.Move(0, spinVal) # spin
	else: 
		Roomba.Move(movSpd, 0) # forward

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PauseQueryStream() # Pause Query Stream before ending program
Roomba.Move(0,0) # Stop Roomba movement
x = Roomba.DirectRead(Roomba.Available()) # Clear buffer
Roomba.PlaySMB()
GPIO.output(gled, GPIO.LOW) # turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program

