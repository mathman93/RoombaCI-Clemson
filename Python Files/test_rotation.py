# Bhavik Suthar and Ayush Petigara
# RoombaCI

import RoombaCI_lib
import RPi.GPIO as GPIO
import sys
import serial
#import time
import math
from RoombaCI_lib import DHTurn

GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)

Roomba.WakeUp(131)
Roomba.BlinkCleanLight()

if Roomba.Available() > 0:
	x = Roomba.DirectRead(Roomba.Available())

# Roomba Constants
WHEEL_DIAMETER = 72 # millimeters
WHEEL_SEPARATION = 235 # millimeters
WHEEL_COUNTS = 508.8 # counts per revolution
DISTANCE_CONSTANT = (WHEEL_DIAMETER * math.pi)/(WHEEL_COUNTS) # millimeters/count
TURN_CONSTANT = (WHEEL_DIAMETER * 180)/(WHEEL_COUNTS * WHEEL_SEPARATION) # degrees/count


l_counts_list = []
r_counts_list = []
angle_list = []
x_pos_list = []
y_pos_list = []
data_time_list = []

final_angle  = 0
#angle = imu.CalculateHeading()
angle = 0
# Initial conditions
distance = 0.0 # total distance traveled (millimeters)
x_pos = 0.0 # initial x-direction position (millimeters)
y_pos = 0.0 # initial y-direction position (millimeters)
forward_value = 0 # initial forward speed value (mm/s)
spin_value = 75 # initial spin speed value (mm/s)
bumper_byte, l_counts_current, r_counts_current, l_speed, r_speed, light_bumper = Roomba.Query(7,43,44,42,41,45) # Read new wheel counts

l_counts_list.append(l_counts_current)
r_counts_list.append(r_counts_current)
angle_list.append(angle)
x_pos_list.append(x_pos)
y_pos_list.append(y_pos)
data_time_list.append(0.0)

Roomba.Move(forward_value, spin_value)
Roomba.StartQueryStream(7,43,44,42,41,45)


init_time = time.time ()
rotation = 360  # in units of degress
while (angel < rotation):
	
	if Roomba.Available() > 0:
		bumper_byte, l_counts, r_counts, l_speed, r_speed, light_bumper = Roomba.ReadQueryStream(7,43,44,42,41,45) # Read new wheel counts

			# Record the current time since the beginning of loop
		data_time = time.time() - init_time
		
		# Calculate the count differences and correct for overflow
		delta_l_count = (l_counts - l_counts_current)
		if delta_l_count > pow(2,15): # 2^15 is somewhat arbitrary
			delta_l_count -= pow(2,16)
		if delta_l_count < -pow(2,15): # 2^15 is somewhat arbitrary
			delta_l_count += pow(2,16)
		delta_r_count = (r_counts - r_counts_current)
		if delta_r_count > pow(2,15): # 2^15 is somewhat arbitrary
			delta_r_count -= pow(2,16)
		if delta_r_count < -pow(2,15): # 2^15 is somewhat arbitrary
			delta_r_count += pow(2,16)
		
		# Calculate the turn angle change since the last counts
		angle_change = TURN_CONSTANT * (delta_l_count - delta_r_count) # degrees
		# Update angle of Roomba and correct for overflow
		angle += angle_change # degrees
		if angle >= 360 or angle < 0:
			angle = (angle % 360) # Normalize the angle value from [0,360)
		
		# Calculate the distance change since the last counts
		if delta_l_count == delta_r_count: # or if angle_change == 0
			# Straight Line distance
			distance_change = 0.5 * DISTANCE_CONSTANT * (delta_l_count + delta_r_count) # millimeters
			# Total distance traveled
			distance += distance_change # millimeters
		else: # Circular Arc distance
			distance_radius = WHEEL_SEPARATION * ((delta_l_count/(delta_l_count - delta_r_count)) - 0.5) # millimeters
			distance_change = 2 * distance_radius * math.sin(0.5 * math.radians(angle_change)) # millimeters
			# Total distance traveled
			distance += (distance_radius * math.radians(angle_change)) # millimeters; Slightly larger than distance_change
		
		# Calculate position data
		delta_x_pos = distance_change * math.cos(math.radians(angle - (0.5 * angle_change)))
		delta_y_pos = distance_change * math.sin(math.radians(angle - (0.5 * angle_change)))
		x_pos += delta_x_pos
		y_pos += delta_y_pos

		final_angle = angle



		#appending to the lists
		l_counts_list.append(l_counts)
		r_counts_list.append(r_counts)
		angle_list.append(angle)
		x_pos_list.append(x_pos)
		y_pos_list.append(y_pos)
		data_time_list.append(data_time)

		l_counts_current = l_counts
		r_counts_current = r_counts
	#end if roomba.available > 0
#end while loop
Roomba.Move(0,0)
time.sleep(0.5)
print("Roomba GOING STRAIGHT TESTING", file=open("outputRotation.txt","a"))
print("\nL Count", file=open("outputRotation.txt","a"))
for i in range(len(l_counts_list)):
	print("{:.3f}".format(l_counts_list[i]), file=open("outputRotation.txt","a"), end="")
	print(", ", file=open("outputRotation.txt","a"), end="")

print("\nR Count", file=open("outputRotation.txt","a"))
for i in range(len(r_counts_list)):
	print("{:.3f}".format(r_counts_list[i]), file=open("outputRotation.txt","a"), end="")
	print(", ", file=open("outputRotation.txt","a"), end="")

print("\nAngle", file=open("outputRotation.txt","a"))
for i in range(len(angle_list)):
	print("{:.3f}".format(angle_list[i]), file=open("outputRotation.txt","a"), end="")
	print(", ", file=open("outputRotation.txt","a"), end="")

print("\nX-Pos", file=open("outputRotation.txt","a"))
for i in range(len(x_pos_list)):
	print("{:.3f}".format(x_pos_list[i]), file=open("outputRotation.txt","a"), end="")
	print(", ", file=open("outputRotation.txt","a"), end="")

print("\nY-Pos", file=open("outputRotation.txt","a"))
for i in range(len(y_pos_list)):
	print("{:.3f}".format(y_pos_list[i]), file=open("outputRotation.txt","a"), end="")
	print(", ", file=open("outputRotation.txt","a"), end="")	

print("\nDate Time", file=open("outputRotation.txt","a"))
for i in range(len(data_time_list)):
	print("{:.3f}".format(data_time_list[i]), file=open("outputRotation.txt","a"), end="")
	print(", ", file=open("outputRotation.txt","a"), end="")	

print("\nFinal Angle: ", final_angle, file=open("outputRotation.txt", "a"))		
print("\nFinal Angle: ", final_angle)

Roomba.ShutDown()
