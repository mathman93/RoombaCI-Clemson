## Import libraries ##
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
	# Give a path
	return


'''Next point to travel towards along path'''
def moveNext(start,end,position):

	# magpos = math.sqrt((position[0]-start[0])**2+(position[1]-start[1])**2)
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
	# Roomba heading = Roomba.heading
	# Calculate heading for roomba
	magnext = math.sqrt((next[0]-position[0])**2+(next[1]-position[1])**2)
	theta_1 = math.acos(next[0]/magnext)
	theta = Roomba.heading
	theta_turn = math.sin(theta_1-theta)
	# do we want to turn left or right
	if(theta_turn > 0.05):
		t_dir = 1
	elif(theta_turn < 0.05):
		t_dir = -1
	# used to find if roomba is going to turn before or while roomba is moving
	theta_sb = math.cos(theta_1-theta)
	# turn fast and move forward slow
	if(math.pi*-7/12 < theta_sb < math.pi*5/12):
		forwardspeed = 20
		spinspeed = 60
	# turn medium and move forward medium
	elif(math.pi*-3/12 < theta_sb < math.pi*3/12):
		forwardspeed = 40
		spinspeed = 20
	# do not need to turn and move forward only
	elif(math.pi/48 < theta_sb < math.pi/48):
		forwardspeed = 100
	# angle is too far away and need to turn before moving
	else:
		spinspeed = 100
	moveList = [forwardspeed,spinspeed*t_dir]
	return moveList
	
# End futurePoint



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