## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math
import heapq
r = 350
''' Predicts the future point that the roomba will be at'''
def futurePoint(velocity,pos):
	# velocity = [speed,angle]
	time = (235/2)/velocity[0]
	xpos = pos[0] + velocity[0] * math.cos(velocity[1]) * time
	ypos = pos[1] + velocity[0] * math.sin(velocity[1]) * time
	futurePos = [xpos,ypos]
	return futurePos
# end futurePoint

# Use dot products and unit vector to find distance along path.
# Calculate target point a certain amount ahead of this distance along the path
# Use this point and current position to find the heading for the roomba
# Have roomba rotate if too far from this angle and then have it start moving towards target
# Continue doing this until target passes the endpoint and make the target the endpoint

'''Defining a Path'''
def path():

'''Distance to Path'''
def distancePath(path,x_pos,y_pos):

'''Distance of future point from path'''
def distanceFuture(path,x_pos,y_pos):



# End futurePoint
# 	
# The above code should calculate both the future point and the distance away from the path


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