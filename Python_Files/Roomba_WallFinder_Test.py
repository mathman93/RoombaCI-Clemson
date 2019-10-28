'''
Roomba_WallFinder_Test.py
Purpose: Test code to have Roomba report when it has hit an obstacle
Last Modified: 10/28/19
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
# may need more later

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

#initialize move speed - necessary?
#movSpd = 138
#spnspd = 100

# initalize values
bumper_byte = 0
# others as necessary later

# Main Code #
query_time = time.time() # set base time for query
query_time_offset = 5*(0.015) # set time offset for query
# smallest time offset for query is 15 ms

Roomba.Move(0,0) # Start Roomba moving

Roomba.StartQueryStream(7) # Start query stream with specific sensor packets
# can add other packets later if needed
while True:
	#try:
	if Roomba.Available() > 0:
		bumper_byte = Roomba.ReadQueryStream(7)
		print("{0:0>8b}".format(bumper_byte))

		# Bumper logic
		if (bumper_byte % 4) > 0:	# if there is a hit to bumper
			Roomba.Move(0,0) # stop Roomba; replace later
			#Roomba.PlaySMB() #check
			if (bumper_byte % 4) == 1:
				# right bump
				print(" Right bumper hit!") #check
			elif(bumper_byte % 4) == 2:
				# left bump
				print(" Left bumper hit!") #check
                        #else:
				# both - front hit
				#print(" Hit head on!") #check
		#else:
			#no hit
			#print(" Still clear!") # for debugging
	#except:
		#print('') # print new line)
		#break # exit while loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PauseQueryStream() # Pause Query Stream before ending program
#Roomba.Move(0,0) # Stop Roomba movement
x = Roomba.DirectRead(Roomba.Available()) # Clear buffer
Roomba.PlaySMB()
GPIO.output(gled, GPIO.LOW) # turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program

