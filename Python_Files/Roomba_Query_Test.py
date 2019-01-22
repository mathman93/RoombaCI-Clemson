''' Roomba_Query_Test.py
Purpose: Testing Query functions from the RoombaCI_lib
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/27/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

import RoombaCI_lib

## Variables and Constants ##
#global Xbee # Specifies connection to Xbee
#Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
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
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
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
	x = Roomba.DirectRead(Roomba.Available()).decode() # Clear out Roomba boot-up info
	print(x) # Include for debugging

print(" ROOMBA Setup Complete")
time.sleep(1)

# Main Code #
''' Query Testing
	'''
'''query_timer = (1/64) # ~ 0.015625
query_base = time.time()
while True:
	try:
		if (time.time() - query_base) > query_timer:
			num = Roomba.SendQuery(7,43,44,42,41,45) 
			query_base += query_timer
			start_time1 = time.time()
		
		if Roomba.Available() > 0:
			stop_time1 = time.time()
			bumper_byte, l_counts, r_counts, l_speed, r_speed, light_bumper = Roomba.ReadQuery(7,43,44,42,41,45)
			
			print(stop_time1 - start_time1)
			print("{0}, {1}, {2:0>8b}, {3:0>8b}, {4}, {5};".format(l_counts, r_counts, bumper_byte, light_bumper, l_speed, r_speed))
		
		
	except KeyboardInterrupt:
		break
'''

''' QueryStream Testing
	'''
Roomba.StartQueryStream(7,43,44,42,41,45)
#Roomba.StartQueryStream(7,45)
start_time1 = time.time()
while True:
	try:
		
		if Roomba.Available() > 0:
			stop_time1 = time.time()
			start_time2 = time.time()
			bumper_byte, l_counts, r_counts, l_speed, r_speed, light_bumper = Roomba.ReadQueryStream(7,43,44,42,41,45)
			#bumper_byte, light_bumper = Roomba.ReadQueryStream(7,45)
			stop_time2 = time.time()
			
			print("{0}, {1}, {2:0>8b}, {3:0>8b}, {4}, {5}, {6:.6f}, {7:.6f};".format(l_counts, r_counts, bumper_byte, light_bumper, l_speed, r_speed, stop_time1 - start_time1, stop_time2 - start_time2))
			#print("{0:0>8b}, {1:0>8b}, {2:.6f}, {3:.6f};".format(bumper_byte, light_bumper, stop_time1 - start_time1, stop_time2 - start_time2))
			start_time1 = time.time()
		else:
			pass
		
	except KeyboardInterrupt:
		break

Roomba.PauseQueryStream()
if Roomba.Available() > 0:
	x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
	#print(x) # Include for debugging purposes

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
#Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
