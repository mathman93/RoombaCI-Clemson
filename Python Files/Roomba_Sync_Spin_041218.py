''' Roomba_Sync_Spin.py
Purpose: Synchronize heading of Roomba network using PCO model
	Based off Arduino code from previous semester
	INCOMPLETE: Needs magnetometer reading code
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 5/23/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

#from Roomba_lib import * # Not sure if this works yet.

## Variables and Constants ##
global Roomba # Specifies connection to Roomba
global Xbee # Specifies connection to Xbee
Roomba = serial.Serial('/dev/ttyS0', 115200) # Baud rate should be 115200
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 57600
# LED pin numbers
yled = 5
rled = 6
gled = 13
## Roomba DD pin
ddPin = 23

# Pulse definitions
global reset_pulse = "b" # Rest pulse character
global sync_pulse = "a" # Sync pulse character

# Timing Counter Parameters
global counter_base
global data_base
data_timer = 1.0
global reset_base
reset_timer = 300

# Counter Constants
cycle_time = 10 # Length of PCO cycle in seconds
counter_adjust = cycle_time # Amount of counter adjustment per cycle
counter_ratio = 360/cycle_time # Converts counter seconds into degrees

# Synchronization Parameters
global angle # Heading of Roomba (found from magnetometer)
global counter # Counter of Roomba (works with angle to compute "phase")
coupling_ratio = 0.5 # Ratio for amount to turn - in range (0, 1]
global epsilon = 1.0 # (Ideally) smallest resolution of magnetometer
global DH_flag = False # DHTurn() boolean (was the last command to turn?)
global data_counter = 0 # Data number counter
global desired_heading  # Heading set point for Roomba


## Functions and Definitions ##
''' Converts integers into bytes (Ints To Bytes)
	Only does integers in range [0, 255]'''
def itb(num):
	return (num).to_bytes(1, byteorder='big', signed=False)

''' Blinks the clean button on Roomba during startup
	Helps determine that RPi -> Roomba communication is working'''
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

''' Send command to Roomba to move
	x is common wheel speed (mm/s); y is diffential wheel speed (mm/s)
	x > 0 -> forward motion; y > 0 -> CW motion
	Error may result if |x| + |y| > 500.'''
def Move(x,y):
	RW = x - y # Right wheel speed
	LW = x + y # Left wheel speed
	Roomba.write(itb(145)) # Send command to Roomba to set wheel speeds
	Roomba.write((RW).to_bytes(2, byteorder='big', signed=True))
	Roomba.write((LW).to_bytes(2, byteorder='big', signed=True))

''' You are not expected to understand this. :)
	'''
def PlaySMB():
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

''' Sends sync_pulse to Xbee
	Used to signal when phase equals 360 degrees '''
def SendSyncPulse():
	#print("Sync Pulse Sent") # Include for debugging
	GPIO.output(gled, GPIO.HIGH) # Tells me I'm sending a pulse
	Xbee.write(sync_pulse.encode())
	GPIO.output(gled, GPIO.LOW)  # Tells me I'm done sending a pulse

''' Sends reset_pulse to Xbee
	Used to signal when to new node joins network '''
def SendResetPulse():
	#print("Reset Pulse Sent") # Include for debugging
	GPIO.output(gled, GPIO.HIGH) # Tells me I'm sending a pulse
	GPIO.output(rled, GPIO.HIGH)
	Xbee.write(reset_pulse.encode())
	GPIO.output(gled, GPIO.LOW)  # Tells me I'm done sending a pulse
	GPIO.output(rled, GPIO.LOW)

''' Receives pulse from Xbee and returns the value
	'''
def ReceivePulse():
	if Xbee.inWaiting() > 0:
		message = Xbee.read(1).decode() # Read in one data
		print("Received:", message) # Include for debugging
		return message # Return the data read.
	else:
		return '' # Return empty string

''' Reset all of the time-based counters
	Used when restarting synchronization'''
def ResetCounters():
	counter_base = time.time() # initialize counter
	data_base = time.time() # initialize data timer
	reset_base = time.time() # initialize reset timer
	counter = 0 # Reset phase counter
	data_counter = 0 # Reset data point counter
	DesiredHeading = angle # Reset heading set point
	PrintData() # Print initial data values

''' Returns necessary change in heading when a sync_pulse is received
	'''
def PRCSync(phase):
	if phase > (360 - epsilon):
		angle_change = 0 # No change in heading
		GPIO.output(rled, GPIO.HIGH) # Indicate sync pulse received, but no turning
	elif phase > 180:
		angle_change = (360 - phase) # Increase heading
		GPIO.output(rled, GPIO.LOW) # Indicate sync pulse received caused turn
	elif phase > epsilon:
		angle_change = (-1) * phase # Decrease heading
		GPIO.output(rled, GPIO.LOW) # Indicate sync pulse received caused turn
	else:
		angle_change = 0 # No change in heading
		GPIO.output(rled, GPIO.HIGH) # Indicate sync pulse received, but no turning
	return angle_change

''' Prints global variables to monitor
	Increments data_counter
	Will need to include magnetometer data '''
def PrintData(*argv):
	global data_counter
	# Print data to console in MATLAB format
	print(data_counter, *argv, sep=', ', end=';\n')
	data_counter += 1 # Increment data counter

''' Sets Roomba spin to achieve desired heading set point
	'''
def DHTurn():
	if (angle < (desired_heading + epsilon) and angle > (desired_heading - epsilon) and DH_flag = False):
		# Roomba is not moving, and it's close to the heading set point
		GPIO.output(yled, GPIO.LOW) # Says we have stopped turning
		return
	
	thresh_1 = 25 # First threshold value
	thresh_2 = 5  # Second threshold value
	
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
			Move(0,-spin_value) # Spin Left (CCW)
			DH_flag = True
		elif (angle < (360 + desired_heading - epsilon)): # and angle >= (desired_heading + 180) 
			Move(0,spin_value) # Spin Right (CW)
			DH_flag = True
		else: # if (360 + desired_heading - epsilon) < angle < (desired_heading + epsilon)
			Move (0,0) # Stop Spinning
			DH_flag = False
			GPIO.output(yled, GPIO.LOW) # Says we have stopped turning
	elif desired_heading < 180: # and desired_heading >= epsilon...
		if (angle > (desired_heading + epsilon) and angle < (desired_heading + 180)):
			Move(0,-spin_value) # Spin Left (CCW)
			DH_flag = True
		elif (angle < (desired_heading - epsilon) or angle >= (desired_heading + 180)):
			Move(0,spin_value) # Spin Right (CW)
			DH_flag = True
		else: # if (desired_heading - epsilon) < angle < (desired_heading + epsilon)
			Move (0,0) # Stop Spinning
			DH_flag = False
			GPIO.output(yled, GPIO.LOW) # Says we have stopped turning
	elif desired_heading < (360 - epsilon):
		if (angle < (desired_heading - epsilon) and angle > (desired_heading - 180)):
			Move(0,spin_value) # Spin Right (CW)
			DH_flag = True
		elif (angle > (desired_heading + epsilon) or angle <= (desired_heading - 180)):
			Move(0,-spin_value) # Spin Left (CCW)
			DH_flag = True
		else: # if (desired_heading - epsilon) < angle < (desired_heading + epsilon) 
			Move (0,0) # Stop Spinning
			DH_flag = False
			GPIO.output(yled, GPIO.LOW) # Says we have stopped turning
	else: # if desired_heading >= (360 - epsilon)
		if (angle < (desired_heading - epsilon) and angle > (desired_heading - 180)):
			Move(0,spin_value) # Spin Right (CW)
			DH_flag = True
		elif (angle > (desired_heading + epsilon - 360)): # and (angle <= (desired_heading - 180))
			Move(0,-spin_value) # Spin Left (CCW)
			DH_flag = True
		else: # if (angle > (desired_heading - epsilon) or angle < (desired_heading + epsilon - 360))
			Move (0,0) # Stop Spinning
			DH_flag = False
			GPIO.output(yled, GPIO.LOW) # Says we have stopped turning

''' Displays current date and time to the screen
	'''
def DisplayDateTime():
	# Month day, Year, Hour:Minute:Seconds
	date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
	print("Program run: ", date_time)

## -- Code Starts Here -- ##
# Setup Code #
DisplayDateTime() # Display current date and time for program
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO

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

BlinkCleanLight() # Blink the Clean light on the Roomba

if Roomba.inWaiting() > 0: # If anything is in the Roomba receive buffer
	x = Roomba.read(Roomba.inWaiting()).decode() # Clear out Roomba boot-up info
	#print(x) # Include for debugging

# Calibrate Magnetometer
Move(0,-75)
# Do stuff
time.sleep(0.1) # Include for now until we do stuff
Move(0,0)

time.sleep(0.5) #
print(" Setup Complete")
GPIO.output(gled, GPIO.LOW) # Says we've finished setup

if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging

# Main Code #

# Initialize Synchronization
angle = 0 # Get initial heading information
SendResetPulse() # Send reset pulse
ResetCounters() # Reset counter values

while True:		
	try:
		# Get heading of Roomba
		angle = 0
		# Set counter value
		counter = (time.time() - counter_base)*counter_ratio
		# Send sync_pulse
		if (angle + counter) > 360: # If (angle + counter) is greater than 360 degrees...
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
			# maybe clear out message variable?
		elif message == sync_pulse:
			print("Sync Pulse Received.") # Include for debugging
			GPIO.output(yled, GPIO.HIGH) # Says we have received sync_pulse
			# Stays on until Roomba stops turning
			
			d_angle = PRCSync(angle + counter) # Calculate desired change in heading
			desired_heading = angle + (d_angle * coupling_ratio) # Update desired heading
			# Normalize desired_heading to range [0,360)
			if desired_heading < 0: # If negative,
				desired_heading += 360 # add 360 degrees
			elif deisred_heading >= 360: # If greater than 360 degrees,
				desired_heading -= 360 # subtract 360 degrees
			# maybe clear out message variable?
		
		DHTurn() # Turn to desired heading point
		
		# Request data packets from Roomba (May use Query Stream)
		
		# Receive data from Roomba
		#if Roomba.inWaiting() > 0: # If packets are received
			
		
		# Print heading data to monitor every second
		if (time.time() - data_base) > data_timer: # After one second
			PrintData()
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
Move(0,0) # Stop Roomba
PlaySMB()

Roomba.write(itb(128)) # Send Roomba to Passive Mode
Roomba.write(itb(174)) # STOP Roomba OI
time.sleep(0.05)
Roomba.close() # Close the Roomba serial port.
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program
