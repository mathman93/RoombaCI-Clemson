''' Roomba_Sync_Spin_041218.py
Purpose: Synchronize heading of Roomba network using PCO model
	Based off Arduino code from previous semester
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 5/31/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

import Roomba_lib # Make sure this file is in the same directory
import IMU_lib # Make sure this file is in the same directory

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

# Pulse definitions
reset_pulse = "b" # Rest pulse character
sync_pulse = "a" # Sync pulse character

# Timing Counter Parameters
data_timer = 1.0
reset_timer = 300

# Counter Parameters
cycle_threshold = 360.0 # Threshold for phase of PCO
cycle_time = 10.0 # Length of PCO cycle in seconds

# Counter Constants
counter_adjust = cycle_time # Amount of counter adjustment per cycle
counter_ratio = (cycle_threshold)/(cycle_time) # Fraction of phase cycle completed in one second

# Synchronization Parameters
global angle # Heading of Roomba (found from magnetometer)
global counter # Counter of Roomba (works with angle to compute "phase")
coupling_ratio = 0.5 # Ratio for amount to turn - in range (0, 1]
epsilon = 1.0 # (Ideally) smallest resolution of magnetometer
global desired_heading  # Heading set point for Roomba

refr_period = 0.0*cycle_threshold # Refractory period for PRC 

## Functions and Definitions ##
''' Sends sync_pulse to Xbee
	Used to signal when phase equals 360 degrees '''
def SendSyncPulse():
	global sync_pulse
	global gled
	#print("Sync Pulse Sent") # Include for debugging
	GPIO.output(gled, GPIO.HIGH) # Tells me I'm sending a pulse
	Xbee.write(sync_pulse.encode()) # Send pulse over Xbee
	GPIO.output(gled, GPIO.LOW)  # Tells me I'm done sending a pulse

''' Sends reset_pulse to Xbee
	Used to signal when to new node joins network '''
def SendResetPulse():
	global reset_pulse
	global gled
	global rled
	#print("Reset Pulse Sent") # Include for debugging
	GPIO.output(gled, GPIO.HIGH) # Tells me I'm sending a pulse
	GPIO.output(rled, GPIO.HIGH)
	Xbee.write(reset_pulse.encode()) # Send pulse over Xbee
	GPIO.output(gled, GPIO.LOW)  # Tells me I'm done sending a pulse
	GPIO.output(rled, GPIO.LOW)

''' Receives pulse from Xbee and returns the value
	'''
def ReceivePulse():
	if Xbee.inWaiting() > 0:
		message = Xbee.read(1).decode() # Read in one data
		#print("Received:", message) # Include for debugging
		return message # Return the data read.
	else:
		return '' # Return empty string

''' Reset all of the time-based counters
	Used when restarting synchronization'''
def ResetCounters():
	global counter_base
	global data_base
	global reset_base
	global counter
	global data_counter
	global angle
	global desired_heading
	counter_base = time.time() # Initialize counter
	data_base = time.time() # Initialize data timer
	reset_base = time.time() # Initialize reset timer
	counter = 0 # Reset phase counter
	data_counter = 0 # Reset data point counter
	angle = imu.CalculateHeading() # Reset initial angle value
	desired_heading = angle # Set to initial angle value

''' Returns necessary change in heading when a sync_pulse is received
	For standard delay-advance phase response function with refractory period
	'''
def PRCSync(phase):
	global cycle_threshold
	global refr_period
	global rled
	global epsilon
	if phase > refr_period: # If phase is greater than the refractory period...
		if phase > (cycle_threshold - epsilon):
			angle_change = 0 # No change in heading
			GPIO.output(rled, GPIO.HIGH) # Indicate sync pulse received, but no turning
		elif phase > 0.5*(cycle_threshold):
			angle_change = (cycle_threshold - phase) # Increase heading
			GPIO.output(rled, GPIO.LOW) # Indicate sync pulse received caused turn
		elif phase > epsilon:
			angle_change = (-1) * phase # Decrease heading
			GPIO.output(rled, GPIO.LOW) # Indicate sync pulse received caused turn
		else:
			angle_change = 0 # No change in heading
			GPIO.output(rled, GPIO.HIGH) # Indicate sync pulse received, but no turning
	else:
		angle_change = 0 # No change in heading
	return angle_change

''' Sets Roomba spin to achieve desired heading set point
	'''
def DHTurn():
	global angle
	global desired_heading
	global epsilon
		
	thresh_1 = 25 # First threshold value (degrees)
	thresh_2 = 5  # Second threshold value (degrees)
	
	diff = abs(angle - desired_heading)
	# Determine spin speed based on thresholds
	if (diff > thresh_1 and diff < (360 - thresh_1)):
		spin_value = 100 # Move faster when farther away from the set point
	elif (diff > thresh_2 and diff < (360 - thresh_2)):
		spin_value = 50 # Move slower when closer to the set point
	else:
		spin_value = 15 # Move very slow when very close to the set point
		# Reduces oscillations due to magnetometer variation and loop execution rate
	
	# Determine direction of spin
	if desired_heading < epsilon: # if 0 <= desired_heading < epsilon 
		if (angle > (desired_heading + epsilon) and angle < (desired_heading + 180)):
			return -spin_value # Spin Left (CCW)
		elif (angle < (360 + desired_heading - epsilon)): # and angle >= (desired_heading + 180) 
			return spin_value # Spin Right (CW)
		else: # if (360 + desired_heading - epsilon) < angle < (desired_heading + epsilon)
			return 0 # Stop Spinning
	elif desired_heading < 180: # and desired_heading >= epsilon...
		if (angle > (desired_heading + epsilon) and angle < (desired_heading + 180)):
			return -spin_value # Spin Left (CCW)
		elif (angle < (desired_heading - epsilon) or angle >= (desired_heading + 180)):
			return spin_value # Spin Right (CW)
		else: # if (desired_heading - epsilon) < angle < (desired_heading + epsilon)
			return 0 # Stop Spinning
	elif desired_heading < (360 - epsilon):
		if (angle < (desired_heading - epsilon) and angle > (desired_heading - 180)):
			return spin_value # Spin Right (CW)
		elif (angle > (desired_heading + epsilon) or angle <= (desired_heading - 180)):
			return -spin_value # Spin Left (CCW)
		else: # if (desired_heading - epsilon) < angle < (desired_heading + epsilon) 
			return 0 # Stop Spinning
	else: # if desired_heading >= (360 - epsilon)
		if (angle < (desired_heading - epsilon) and angle > (desired_heading - 180)):
			return spin_value # Spin Right (CW)
		elif (angle > (desired_heading + epsilon - 360)): # and (angle <= (desired_heading - 180))
			return -spin_value # Spin Left (CCW)
		else: # if (angle > (desired_heading - epsilon) or angle < (desired_heading + epsilon - 360))
			return 0 # Stop Spinning

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
Roomba = Roomba_lib.Create_2("/dev/ttyS0", 115200)
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
imu = IMU_lib.LSM9DS1_IMU() # Initialize IMU
time.sleep(0.5)
# Calibrate IMU
print(" Calibrating IMU...")
Roomba.Move(0,75) # Start Roomba spinning
imu.CalibrateMag() # Calculate magnetometer offset values
Roomba.Move(0,0) # Stop Roomba spinning
time.sleep(0.5)
imu.CalibrateAccelGyro() # Calculate accelerometer and gyroscope offset values
# Display offset values
print("mx_offset = {:f}; my_offset = {:f}; mz_offset = {:f}".format(imu.mx_offset, imu.my_offset, imu.mz_offset))
print("ax_offset = {:f}; ay_offset = {:f}; az_offset = {:f}".format(imu.ax_offset, imu.ay_offset, imu.az_offset))
print("gx_offset = {:f}; gy_offset = {:f}; gz_offset = {:f}".format(imu.gx_offset, imu.gy_offset, imu.gz_offset))
print(" IMU Setup Complete")
time.sleep(1) # Gives time to read offset values before continuing
GPIO.output(yled, GPIO.LOW) # Indicate setup sequence is complete

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging

# Main Code #

# Initialize Synchronization
angle = imu.CalculateHeading() # Get initial heading information
SendResetPulse() # Send reset pulse
ResetCounters() # Reset counter values
# Request data packets from Roomba (Stream)
Roomba.StartQueryStream(7, 43, 44, 45, 41, 42) # Start query stream with specific sensor packets

while True:		
	try:
		# Get heading of Roomba
		angle = Roomba.CalculateHeading()
		# Set counter value
		counter = (time.time() - counter_base)*counter_ratio
		# Send sync_pulse
		if (angle + counter) > cycle_threshold: # If (angle + counter) is greater than 360 degrees...
			SendSyncPulse()
			counter_base += counter_adjust
			
		# Receive pulse
		message = ReceivePulse()
		
		if message == reset_pulse: 
			print("Reset Pulse Received.") # Include for debugging
			GPIO.output(gled, GPIO.HIGH) # Notify that reset_pulse received
			GPIO.output(rled, GPIO.HIGH)
			ResetCounters() # Reset counters
			GPIO.output(gled, GPIO.LOW)  # End notify that reset_pulse received
			GPIO.output(rled, GPIO.LOW)
		elif message == sync_pulse:
			print("Sync Pulse Received.") # Include for debugging
			d_angle = PRCSync(angle + counter) # Calculate desired change in heading
			desired_heading = angle + (d_angle * coupling_ratio) # Update desired heading
			# Normalize desired_heading to range [0,360)
			if desired_heading < 0: # If negative,
				desired_heading += cycle_threshold # add 360 degrees
			elif deisred_heading >= cycle_threshold: # If greater than 360 degrees,
				desired_heading -= cycle_threshold # subtract 360 degrees
		
		spin = DHTurn() # Value needed to turn to desired heading point
		Roomba.Move(forward, spin) # Move Roomba to desired heading point
		
		if spin == 0:
			GPIO.output(yled, GPIO.LOW) # Indicate Roomba is not turning
		else:
			GPIO.output(yled, GPIO.HIGH) # Indicate Roomba is turning
		
		# Read query stream for specific packets (ReadQueryStream)
		if Roomba.Available() > 0:
			bumper_byte, l_counts, r_counts, light_bumper, r_speed, l_speed = Roomba.ReadQueryStream(7, 43, 44, 45, 41, 42)
		
		# Print heading data to monitor every second
		if (time.time() - data_base) > data_timer: # After one second
			print("{0}, {1:0>8b}, {2}, {3}, {4:0>8b}, {5}, {6};".format(data_counter, bumper_byte, l_counts, r_counts, light_bumper, l_speed, r_speed))
			data_counter += 1 # Increment counter for the next data sample
			data_base += data_timer
		
		# Reset counters of all Roombas after 5 minutes
		if (time.time() - reset_base) >= reset_timer: # After 5 minutes
			SendResetPulse() # Send reset_pulse
			# Reset all counters
			ResetCounters()
		
	except KeyboardInterrupt:
		print('') # print new line
		break # exit while loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.PauseQueryStream() # Pause Query Stream before ending program
Roomba.Move(0,0) # Stop Roomba movement
x = Roomba.DirectRead(Roomba.Available()) # Clear buffer
#Roomba.PlaySMB()
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
