## Import libraries ##
from Python_Files.MapAndMove import Roomba
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math
import heapq


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
	pathV[0] = end[0]-start[0]
	pathV[1] = end[1]-start[1]
	# calculates roomba vector
	roombaV = []
	roombaV[0] = position[0]-start[0]
	roombaV[0] = position[1]-start[1]
	# magnitude of the path vector
	magpath = math.sqrt(pathV[0]**2 + pathV[1]**2)
	# unit vector of the path
	upath = []
	upath[0] = pathV[0]/magpath
	upath[1] = pathV[1]/magpath
	# dot product calculation
	dotp = roombaV[0]*pathV[0]+roombaV[1]*pathV[1]
	# calculates projection to closest point on line
	proj = []
	proj[0] = dotp*pathV[0]/(magpath**2)
	proj[1] = dotp*pathV[1]/(magpath**2)
	# calculates next seek point based on projection
	next = []
	next[0] = proj[0]*1.05
	next[1] = proj[1]*1.05
	# returns the seek point x and y in a list
	return next
# end of seek

def heading(next,position):
	# Roomba heading = Roomba.heading
	# Calculate heading for roomba
	magnext = math.sqrt((next[0]-position[0])**2+(next[1]-position[1])**2)
	theta_1 = math.acos(next[0]/magnext)
	theta = Roomba.heading
	return theta_1-theta

def moveSpeed(theta):
	theta_turn = math.sin(theta)
	# do we want to turn left or right
	if(theta_turn > 0.05):
		t_dir = 1
	elif(theta_turn < 0.05):
		t_dir = -1
	# used to find if roomba is going to turn before or while roomba is moving
	theta_sb = math.cos(theta)
	# turn fast and don't move forward
	if(theta_sb < 0):
		forwardspeed = 0
		spinspeed = 100
	# turn somewhat fast and move forward slightly
	elif(theta_sb < .5):
		forwardspeed = 20
		spinspeed = 80
	# still needs to turn a bit and but can also start moving
	elif(theta_sb < .97):
		forwardspeed = 30
		spinspeed = 60
	# pretty much in line and only needs to move forward
	else:
		forwardspeed = 100
		spinspeed = 0
	moveList = [forwardspeed,spinspeed*t_dir]
	return moveList
# End moveSpeed

# Get the initial wheel enocder values
[left_encoder, right_encoder] = Roomba.Query(43,44)
Roomba.SetWheelEncoderCounts(left_encoder,right_encoder)

# initialize the new and old path
newpath = ()
oldpath = ()
while True:
	# if this is the first path, set old path to (0,0)
	if newpath == ():
		oldpath = (0,0)
	else:
		oldpath = newpath
	# get the new path
	newpath = path()
	if Roomba.Available() > 0:
		# update position
		Roomba.UpdatePosition(left_encoder,right_encoder)
		xpos = Roomba.X_position
		ypos = Roomba.Y_position
		# find seek point
		seekPoint = seek(oldpath,newpath(xpos,ypos))
		# check if next point is past end point
		# if it is go to end point instead
		# if distance(seekpoint) > distance(end point)
		# heading(newpath,(xpos,ypos))
		# else do below
		# find heading
		theta = heading(seekPoint,(xpos,ypos))
		# find movement speeds
		[fspeed,tspeed] = moveSpeed(theta)
		# give the roomba these speeds
		
# Psudeo Code
# user gives input of a path
# Roomba finds where it is in relation to path and the final point on the path
# Starts calculating future point that will get it closer to both end point and the path
# Calculates speed and angle needed to get to that future point
# Starts moving towards the point
# Repeats the above steps until it reaches end point
# End


# starts at first position in list
# look ahead at next point in list
#	if within a tolearance of that
#	repeat above
# 