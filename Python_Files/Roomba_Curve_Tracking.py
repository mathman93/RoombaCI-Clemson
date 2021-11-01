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
	# find heading based on current heading
	# ask how what the heading is based on
	# give speed of turn and speed of roomba based on how far roomba is from heading
	return
	
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
# have four points around the roomba of x-r,x+r,y-r,y+r
# r is a defined constant "radius"
# if > x+r move right if < x-r move left, if neither nothing
# if > y+r move up if < y-r move down, if neither nothing
# if both neither, move to next point