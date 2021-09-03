''' 
Roomba_BumpControl_Test.py
Purpose: Test code to read bump sensors and avoid obstacles
	Does not control properly; needs adjustment to use of lightbumper value
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/6/2019
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import math
import random
import RoombaCI_lib

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
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	#print(x) # Include for debugging

print(" ROOMBA Setup Complete")
GPIO.output(yled, GPIO.HIGH) # Indicate within setup sequence
# Initialize IMU
print(" Starting IMU...")
imu = RoombaCI_lib.LSM9DS1_I2C() # Initialize IMU
time.sleep(0.1)
# Clear out first reading from all sensors
x = imu.magnetic
x = imu.acceleration
x = imu.gyro
# Calibrate IMU
print(" Calibrating IMU...")
Roomba.Move(0,75) # Start Roomba spinning
imu.CalibrateMag() # Calculate magnetometer offset values
Roomba.Move(0,0) # Stop Roomba spinning
time.sleep(0.5)
imu.CalibrateGyro() # Calculate gyroscope offset values
# Display offset values
print("mx_offset = {:f}; my_offset = {:f}; mz_offset = {:f}"\
	.format(imu.m_offset[0], imu.m_offset[1], imu.m_offset[2]))
print("gx_offset = {:f}; gy_offset = {:f}; gz_offset = {:f}"\
	.format(imu.g_offset[0], imu.g_offset[1], imu.g_offset[2]))
print(" IMU Setup Complete")
time.sleep(3) # Gives time to read offset values before continuing
GPIO.output(yled, GPIO.LOW) # Indicate setup sequence is complete

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging

#initializes move speed 
movSpd = 138
#input the speed
spnspd = 100

#times, spin time is from formula
spinTime = (235 * math.pi) / (4 * spnspd)
backTime = 0.5
#initializes timers
dataTimer = time.time()
timer = time.time()
moveHelper = (time.time() - (spinTime + backTime))
#intitialize values
spinVal = 100
moveVal = 0
bumper_byte = 0
light_bumper = 0

# Main Code #
query_time = time.time() # Set base time for query
query_time_offset = 5*(0.015) # Set time offset for query
# smallest time offset for query is 15 ms.
data_counter = 0 # Initialize data counter

Roomba.Move(0,0) # Start Roomba moving

Roomba.StartQueryStream(7, 43, 44, 45, 41, 42) # Start query stream with specific sensor packets
#datafile = open("data_test.txt", "w") # Open a text file for storing data
	# Will overwrite anything that was in the text file previously
while data_counter < 1001: # stop after 1000 data points
	try:
		if (time.time() - timer) > 0.5:
			#flickers green led for checking if it works
			if GPIO.input(gled) == True:
				GPIO.output(gled, 0)
			else:
				GPIO.output(gled, 1)
			timer = time.time()
		
		'''# Request data packet from Roomba (QuerySingle)
		if (time.time() - query_time) > query_time_offset: # If enough time has passed
			bumper_byte = Roomba.QuerySingle(7) # Ask for Bumper byte packet (1 byte)
			print(bumper_byte)
			query_time += query_time_offset # offset query time for next query
		'''
		'''# Request data packets from Roomba (Query)
		if (time.time() - query_time) > query_time_offset: # If enough time has passed
			bumper_byte, l_wheel, r_wheel, light_bumper = Roomba.Query(7, 43, 44, 45) # Ask for specific sensor packets
			# Print data to screen (MATLAB format)
			print("{0}, {1:0>8b}, {2}, {3}, {4:0>8b};".format(data_counter, bumper_byte, l_wheel, r_wheel, light_bumper))
			data_counter += 1 # Increment counter for the next data sample
			query_time += query_time_offset # offset query time for next query
			'''	
		# Read query stream for specific packets (ReadQueryStream)
		if Roomba.Available() > 0:
			bumper_byte, l_counts, r_counts, light_bumper, r_speed, l_speed = Roomba.ReadQueryStream(7, 43, 44, 45, 41, 42)
			angle = imu.CalculateHeading()
			# Print data values out to the monitor
			print("{0}, {1:0>8b}, {2}, {3}, {4:0>8b}, {5}, {6}, {7:.5f};"\
				.format(data_counter, bumper_byte, l_counts, r_counts, light_bumper, l_speed, r_speed, angle))
			# Write data values to a text file
			#datafile.write("{0}, {1:0>8b}, {2}, {3}, {4:0>8b}, {5}, {6};".format(data_counter, bumper_byte, l_counts, r_counts, light_bumper, l_speed, r_speed))
			data_counter += 1 # Increment counter for the next data sample
			# Bumper logic
			if (bumper_byte % 4) > 0:
				moveHelper = time.time()
				#sets speed after left bumper
				if (bumper_byte % 4) == 1:
					spinVal = -spnspd
					moveVal = -100
				#sets speed after left bumper
				elif (bumper_byte % 4) == 2:
					spinVal = spnspd
					moveVal = -100
				#random int to help get out of corners
				else:
					y = random.randint(0, 1)
					spinVal = random.randint(spnspd - 50, spnspd + 50)
					if y == 0:
						spinVal = -spinVal
						
					moveVal = -100
			
			#if the light sensors detect something, say then slow down
			if light_bumper > 0:
				movSpd = 100
				#print("Slow Move")
			else:		
				movSpd = 225
				#print("Fast Move")
			
		#timer for the backward movement, then the spin
		if (time.time() - moveHelper) < backTime:
			Roomba.Move(moveVal, 0)
			#print("Backward")
		elif (time.time() - moveHelper) < (backTime + spinTime):
			Roomba.Move(0, spinVal)
			#print("Spin")
		else:
			Roomba.Move(movSpd, 0)
			#print("Forward")
		
	except KeyboardInterrupt:
		print('') # print new line
		break # exit while loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PauseQueryStream() # Pause Query Stream before ending program
Roomba.Move(0,0) # Stop Roomba movement
x = Roomba.DirectRead(Roomba.Available()) # Clear buffer
Roomba.PlaySMB()
#datafile.close()
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
