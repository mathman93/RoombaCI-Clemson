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

	#magpos = math.sqrt((position[0]-start[0])**2+(position[1]-start[1])**2)
	magend = math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
	uend = []
	uend[0] = end[0]/magend*1.05
	uend[1] = end[1]/magend*1.05
	proj = []
	proj[0] = position[0]*end[0]/magend*uend[0]
	proj[1] = position[1]*end[1]/magend*uend[1]
	# find heading based on current heading
	# ask how what the heading is based on
	# give speed of turn and speed of roomba based on how far roomba is from heading
	
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