## Import libraries ##
from os import supports_follow_symlinks
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math
import heapq

## Variables and Constants ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
# LED pin numbers
yled = 5
rled = 6
gled = 13

# Use dot products and unit vector to find distance along path.
# Calculate target point a certain amount ahead of this distance along the path
# Use this point and current position to find the heading for the roomba
# Have roomba rotate if too far from this angle and then have it start moving towards target
# Continue doing this until target passes the endpoint and make the target the endpoint

'''Defining a Path'''
def path():
	# ask for input of a path
	return # return the path


'''Next point to travel towards along path'''
def seek(start, end, position):
	# calculates path vector
	pathV = []
	pathV.append(end[0]-start[0])
	pathV.append(end[1]-start[1])
	# calculates roomba vector
	roombaV = []
	roombaV.append(position[0]-start[0])
	roombaV.append(position[1]-start[1])
	# magnitude of the path vector
	magpath = math.sqrt(pathV[0]**2 + pathV[1]**2)
	# unit vector of the path
	upath = []
	upath.append(pathV[0]/magpath)
	upath.append(pathV[1]/magpath)
	# dot product calculation
	dotp = roombaV[0]*pathV[0]+roombaV[1]*pathV[1]
	# calculates projection to closest point on line
	proj = []
	proj.append(dotp*pathV[0]/(magpath**2))
	proj.append(dotp*pathV[1]/(magpath**2))
	# closest point on path
	close = []
	close.append(start[0]+proj[0])
	close.append(start[1]+proj[1])
	# calculates next seek point based on projection
	next = (close[0]+upath[0]*50,close[1] + upath[1]*50)
	# returns the seek point x and y in a tuple
	return next
# end of seek

def heading(next,position,roombah):
	# Roomba heading = Roomba.heading
	# Calculate heading for roomb
	y = next[1] - position[1]
	x = next[0] - position[0]
	theta = math.atan2(y,x)
	rh = (((theta-roombah) + math.pi) % (2*math.pi)) - math.pi
	return rh

def moveSpeed(theta):
	# do we want to turn left or right
	t_dir = -1
	if(theta > 0):
		t_dir = 1
	# used to find if roomba is going to turn before or while roomba is moving
	theta_sb = math.cos(theta)
	# turn fast and don't move forward
	if(theta_sb < 0):
		forwardspeed = 0
	# turn somewhat fast and move forward slightly
	elif(theta_sb < .5):
		forwardspeed = 50
	# still needs to turn a bit and but can also start moving
	elif(theta_sb < .90):
		forwardspeed = 100
	# pretty much in line and only needs to move forward
	else:
		forwardspeed = 150
	
	# use theta to determine the spin speed
	if abs(theta) > math.pi/2:
		spinspeed = 100
	elif abs(theta) > math.pi/4:
		spinspeed = 75
	elif abs(theta) > math.pi/12:
		spinspeed = 50
	elif abs(theta) > 0.05:
		spinspeed = 20
	else:
		spinspeed = 0
	moveList = [forwardspeed,spinspeed*t_dir]
	return moveList
# End moveSpeed

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
RoombaCI_lib.DisplayDateTime() # Display current date and time

ack = input("Do you want to save data to a file (y/n)? ") # user acknowledgement
if ack in ["y", "Y"]: # If positive
	file_create = True # create and save data to file
	file_name_input = input("Name for data file: ") # Ask user for desired file name
	dir_path = "/home/pi/RoombaCI-Clemson/Data_Files/2021_Fall/" # Directory path to save file
	file_name = os.path.join(dir_path, file_name_input+".txt") # text file extension
	datafile = open(file_name, "w") # Open a text file for storing data
		# Will overwrite anything that was in the text file previously
else: # otherwise
	file_create = False # Skip data creation
# End if

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
# End if Roomba.Available()
print(" ROOMBA Setup Complete")
'''
GPIO.output(yled, GPIO.HIGH) # Indicate within setup sequence
# Initialize IMU
print(" Starting IMU...")
imu = RoombaCI_lib.LSM9DS1_I2C() # Initialize IMU
time.sleep(0.2)
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
GPIO.output(yled, GPIO.LOW) # Indicate IMU setup sequence is complete
'''
if Xbee.inWaiting() > 0: # If anything is in the Xbee receive buffer
	x = Xbee.read(Xbee.inWaiting()).decode() # Clear out Xbee input buffer
	#print(x) # Include for debugging
# End if Xbee.inWaiting()
GPIO.output(gled, GPIO.LOW) # Indicate all set sequences are complete

# Main Code ##

# initialize the new and old path
pathpoints = [(326,735),(-654,358),(-845,-356),(985,-423),(786,548),(0,0)]
#prev = (-1000,1000)
#nextpoint = (-1000,-1000)
# Get the initial wheel enocder values
[left_encoder, right_encoder] = Roomba.Query(43,44)
Roomba.SetWheelEncoderCounts(left_encoder,right_encoder)
Roomba.StartQueryStream(43,44)
data_timer = time.time()

for i in range(len(pathpoints)):
	# slows the roomba down the closer it gets to stop point
	fspeedcoeff = 1
	startnext = 1
	nextpoint = pathpoints[i]
	print(nextpoint)
	if i == 0:
		prev = (0,0)
	else:
		prev = pathpoints[i-1]
	#Roomba.StartQueryStream(43,44)
	while startnext:
		try:
			if Roomba.Available() > 0:
				[left_encoder,right_encoder] = Roomba.ReadQueryStream(43,44)
				data_time = time.time()-data_timer
				# update position
				Roomba.UpdatePosition(left_encoder,right_encoder)
				xpos = Roomba.X_position
				ypos = Roomba.Y_position
				# find seek point
				seekPoint = seek(prev,nextpoint,(xpos,ypos))
				# check if next point is past end point
				# seek distance
				dseek = math.sqrt((seekPoint[0]-prev[0])**2+(seekPoint[1]-prev[1])**2)
				# end distance
				dend = math.sqrt((nextpoint[0]-prev[0])**2+(nextpoint[1]-prev[1])**2)
				# position of roomba distance
				dpos = math.sqrt((xpos-prev[0])**2+(ypos-prev[1])**2)
				# changes the speed of the roomba if it is close to next point (within 85%)
				if dpos * 20/17 > dend:
					fspeedcoeff = 1 - dpos/dend + .15
				# if it is go to end point instead
				if dseek > dend:
					theta = heading(nextpoint,(xpos,ypos),Roomba.heading)
					# if this is the last point on the path
					if i == len(pathpoints)-1:
						# if it is slightly past end point
						if dpos > dend:
							print("Finished with path.\n")
							startnext = 0
					# else print position and say its finished with point and go to next point
					else:
						print(xpos,ypos)
						print("Finished with point.\n")
						startnext = 0
				else:
					theta = heading(seekPoint,(xpos,ypos),Roomba.heading)
				# find movement speeds
				[fspeed,tspeed] = moveSpeed(theta)
				# give the roomba these speeds
				Roomba.Move((int)(fspeed*fspeedcoeff),tspeed)

				if file_create == True:
					# Write data values to a text file (MATLAB format)
					datafile.write("{0:.6f}, {1:.3f}, {2:.3f}, {3:.4f}, {4:.3f}, {5:.3f}, {6:.3f}, {7:.3f}\n"\
						.format(data_time, Roomba.X_position, Roomba.Y_position, Roomba.heading, seekPoint[0], seekPoint[1], nextpoint[0], nextpoint[1]))
				# End if

		except KeyboardInterrupt:
			break
	# end while loop
# prints final position roomba ended at
print("Final Position")
print(xpos,ypos)	
Roomba.Move(0,0)
Roomba.PauseQueryStream() # Pause Query Stream before ending program
if Roomba.Available() > 0:
	x = Roomba.DirectRead(Roomba.Available()) # Clear out residual Roomba data
	#print(x) # Include for debugging purposes

if file_create == True:
	datafile.close()
	Roomba.PlaySMB()
# End if
Roomba.ShutDown() # Shutdown Roomba serial connection
Xbee.close()
GPIO.cleanup() # Reset GPIO pins for next program