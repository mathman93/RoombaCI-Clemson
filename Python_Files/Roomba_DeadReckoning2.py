''' Roomba_DeadReckoning2.py
Purpose: Navigate to desired goal point by detecting and avoiding obstacles
	Uses a node-based graph to map possible waypoints and obstacles
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/6/2019
Originally coded by Jay and Akhil A., Tres Y.
Updated by Timothy A.
STILL NEEDS COMMENTING (do not remove this too soon!)
Implement directional cost in A* algorithm
Double check query timers and encoder data collection
'''

## Import Libraries ##
import math
import time
import RPi.GPIO as GPIO
import random
import heapq

import RoombaCI_lib
# Used directly, so called into the program directly.
from RoombaCI_lib import DHTurn
from RoombaCI_lib import DDSpeed

## Variables and Constants ##
x_pos = 0 # Current x coordinate locatoin in millimeters. Initialized to 0.
y_pos = 0 # Current y coordinate locatoin in millimeters. Initialized to 0.

#d_cost_weight = 1 # Multiplies the d_weights by a cost. Can be modified to give less or more weight to the directional cost.

# The value that the Roomba moves by after using the move_step for both move and turn. They make the Roomba move slowly change velocity to avoid jerks.
actual_move = 0
actual_spin = 0
move_step = 20 # The rate at which the Roomba changes speed while moving forward and spinning.

# Bump Detection Timing Constants
back_time = 0.5 # Time (seconds) that Roomba backs up after a bump is detected.
spin_time = 0.0 # Time (seconds) that Roomba spins after backing up
turn_direction = 0 # Used to indicate which direction to spin; 0 = no turn, 1 = turn left, 2 = turn right
break_time = 0.1 # Time (seconds) that Roomba takes to stop following the path
# and perform necessary computations to modify the world points after a bump is detected

# Algorithm Booleans
take_a_break = False # Used to halt execution of path following and recalculate A* when an obstacle is detected
can_do_order_66 = True # Used to remove the first wall_point detected, and keep it from being removed twice.
can_form_neighbors = True # True when two world points can be neighbors

# LED pin numbers
yled = 5
rled = 6
gled = 13

# Roomba Navigation Constants
ROOMBA_RADIUS = 175 # millimeters
WHEEL_DIAMETER = 72 # millimeters
WHEEL_SEPARATION = 235 # millimeters
WHEEL_COUNTS = 508.8 # counts per revolution
DISTANCE_CONSTANT = (WHEEL_DIAMETER * math.pi)/(WHEEL_COUNTS) # millimeters/count
TURN_CONSTANT = (WHEEL_DIAMETER * 180)/(WHEEL_COUNTS * WHEEL_SEPARATION) # degrees/count

## Functions and Definitions ##
''' Priority Queue Class
	Based on code implementation found at https://www.redblobgames.com/pathfinding/a-star/implementation.html '''
class Queue:
	# Creates a queue list on initialization.
	def __init__(self):
		self.elements = []
	
	# Returns true if the Queue is empty.
	def empty(self):
		return len(self.elements) == 0 # True when self.elements is an empty list
	
	# Gives an item and a priority to the list and reorders it based on priority.
	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))
	
	# Returns the top item in the heap.
	def get(self):
		return heapq.heappop(self.elements)[1]

# A* Cost Functions #
''' Calculate distance between two world points
	Used to calculate g-cost in A* algorithm
	Parameters:
		a, b = tuple; x,y world points (order is arbitrary)
	Returns:
		float; Euclidean distance between the two x,y points'''
def g_cost(a, b):
	(x1, y1) = a
	(x2, y2) = b
	return math.sqrt((pow((x1-x2), 2))+(pow((y1-y2), 2)))

''' Calculate h-cost in A* algorithm
	Parameters:
		goal, a = tuple; x,y world points (order is arbitrary)
	Returns:
		float; Euclidean distance between the two x,y points
	Identical caluclation as in g_cost '''
def h_cost(goal, a):
	(x1, y1) = (goal)
	(x2, y2) = a
	return math.sqrt((pow((x1-x2), 2))+(pow((y1-y2), 2)))

''' Uses the A* search algorithm to find and return a path to the goal
	Parameters:
		neighbors = dict; dictionary of neighbor lists for each world point
		start_point = tuple; starting x,y location
		goal = tuple; ending x,y location
			Note: start_point and goal must have entries in neighbors
	Returns:
		path = list; ordered list of world points to travel from start_point to goal
			Note: An empty list is returned if no path is found from start_point to goal
	Based on code implementation found at https://www.redblobgames.com/pathfinding/a-star/implementation.html
'''
def a_star(neighbors, start_point, goal):
	frontier = Queue() # Define a frontier as a Priority Queue
	frontier.put((start_point),0) # Put the start point in with a priority of zero.
	came_from = {} # Define a dictionary that records where we came from to get to that entry.
	cost_so_far = {} # Define a dictionary that gives total cost to reach that entry.
	came_from[(start_point)] = None # Did not come from anywhere to get to the start_point.
	cost_so_far[start_point] = 0 # No initial cost to get where we start.
	# Loop through the frontier until you find the shortest path (or have searched the entire map).
	while not frontier.empty(): # While the frontier queue is not empty...
		current = frontier.get() # Get the top point from the queue to search
		if current == goal: # If where we are searching is where we want to be...
			break # Exit while loop; we made it!
		for next in neighbors[current]: # For each neighboring point of current
			d_cost = 0 # Directional cost (Not yet implemented)
			# Total cost to reach next from current
			new_cost = cost_so_far[current] + g_cost(current, next) + d_cost
			# If the world point, next, is not in the cost_so_far dictionary,
			# or if the new cost is smaller than the previous cost found for next...
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost # Set new_cost for next in the dictionary
				priority = new_cost + h_cost(goal, next) # Incorporated the h-cost into the total cost
				frontier.put(next, priority) # Place next into the frontier queue with its priority
				came_from[next] = current # We came from current to get to next
	
	# Use came_from to make list of points to get from one point to another.
	path = [] # Initially empty list for world points in the desired path
	path.append(goal) # Add goal to path (build the list up backwards).
	
	while start_point not in path: # If start_point is in path, we've reached the end.
		last = path[-1] # Take the last point in path
		path.append(came_from[last]) # Add the point that we came from to get to last
	
	path.reverse() # Path was built backwards, so reverse it.
	del path[0] # Delete the start point; we are already there
	
	# Return the path
	return path

''' Completely removes a world point from the neighbors dictionary
	Used to remove world points that are too close to a wall point
	Parameters:
		wall_point = tuple; world point to be removed
		neighbors = dict; dictionary of neighbor lists for each world point
	Returns:
		neighbors = dict; updated dictionary with updated lists and wall_point entry removed
'''
def Order66(wall_point, neighbors):
	for square in neighbors[wall_point]: # For each entry in the list for wall_point
		# Remove wall_point from the list for that entry
		neighbor_remover = neighbors[square]
		neighbor_remover.remove(wall_point)
		neighbors[square] = neighbor_remover
	del neighbors[wall_point] # Remove the wall_point entry from the dictionary
	print ('Removed',wall_point,'...') # Include for debugging
	return neighbors # Return the updated neighbors dictionary

''' Removes two world points from each other's neighbor list
	Used to remove connection between two world points when a wall is detected.
	Parameters:
		point1, point2 = tuple; first and second world point
			Note: order does not matter for point1 and point2
		neighbors = dict; dictionary of neighbor lists for each world point
			Note: A entry for point1 and point2 should already exist in the neighbors dictionary
	Returns:
		neighbors = dict; updated dictionary with updated lists for point1 and point2
	Error results if point1 or point2 is not in the other's neighbors list
	This function is the opposite of Create_Neighbors '''
def Order65(point1, point2, neighbors):
	# Remove point1 from neighbors list of point2
	neighbor_remover = neighbors[point2]
	neighbor_remover.remove(point1)
	neighbors[point2] = neighbor_remover
	# Remove point2 from neighbors list of point1
	neighbor_remover = neighbors[point1]
	neighbor_remover.remove(point2)
	neighbors[point1] = neighbor_remover
	# Return updated neighbors dictionary
	return neighbors

''' Finds shortest distance from close to a line segment between point1 and point2
	Used to determine if two world points can be connected together without inersecting a wall
	Parameters:
		point1, point2 = tuple; world points that define a line segment
			Note: order does not matter for point1 and point2 
		close = tuple; wall point to find closest distance to
			Note: point3 must be the non line segment point.
	Returns:
		distance = float; shortest distance from close to line segment between point1 and point2
'''
def Intersection_Finder(point1, point2, close):
	# Separate the pair of values in each tuple
	(x1, y1) = point1
	(x2, y2) = point2
	(x3, y3) = close
	# Algebra calculations to determine the intersection point between the line formed 
	# using point1 and point2 and a perpendicular line through close.
	c = x2 - x1
	d = y2 - y1
	b1 = (y1 * c) - (x1 * d)
	b2 = (y3 * d) + (x3 * c)
	x_intersect = ((c * b2) - (d * b1)) / (pow(d, 2) + pow(c, 2)) # Solution for x-coordinate of intersection point
	y_intersect = ((c * b1) + (d * b2)) / (pow(d, 2) + pow(c, 2)) # Solution for y-coordinate of intersection point
	intersection_point	= (x_intersect, y_intersect) # Intersection point tuple
	
	# Determine if intersection_point lies between point1 and point2
	if x1 != x2:
		if x1 < x2:
			if x1 < x_intersect and x_intersect < x2: # If it is between...
				# Shortest distance from close to the line is to intersection_point
				distance = g_cost(intersection_point, close)
			else:
				# Shortest distance is from close to one of the end points
				distance = min(g_cost(point1, close), g_cost(point2, close))
		else:
			if x2 < x_intersect and x_intersect < x1: # If it is between...
				# Shortest distance from close to the line is to intersection_point
				distance = g_cost(intersection_point, close)
			else:
				# Shortest distance is from close to one of the end points
				distance = min(g_cost(point1, close), g_cost(point2, close))
	else: # if x1 == x2, use y-values
		if y1 < y2:
			if y1 < y_intersect and y_intersect < y2: # If it is between...
				# Shortest distance from close to the line is to intersection_point
				distance = g_cost(intersection_point, close)
			else:
				# Shortest distance is from close to one of the end points
				distance = min(g_cost(point1, close), g_cost(point2, close))
		else:
			if y2 < y_intersect and y_intersect < y1: # If it is between...
				# Shortest distance from close to the line is to intersection_point
				distance = g_cost(intersection_point, close)
			else:
				# Shortest distance is from close to one of the end points
				distance = min(g_cost(point1, close), g_cost(point2, close))
	# Return the shortest distance
	return distance

''' Adds two world points into each other's neighbor list
	Parameters:
		point1, point2 = tuple; first and second world point
			Note: order does not matter for point1 and point2
		neighbors = dict; dictionary of neighbor lists for each world point
			Note: A entry for point1 and point2 should already exist in the neighbors dictionary
	Returns:
		neighbors = dict; updated dictionary with updated lists for point1 and point2
	This function is the opposite of Order65 '''
def Create_Neighbors(point1, point2, neighbors):
	# Add point2 to neighbors list of point1
	neighbor_list = neighbors[point1] # Get neighbor list from the dictionary
	neighbor_list.append(point2) # Add point2 to the list
	neighbor_set = set(neighbor_list) # Convert list into set to remove duplicates
	neighbors[point1] = list(neighbor_set) # Reconvert set back into list for point1
	# Add point1 to neighbors list of point2
	neighbor_list = neighbors[point2] # Get neighbor list from the dictionary
	neighbor_list.append(point1) # Add point1 to the list
	neighbor_set = set(neighbor_list) # Convert list into set to remove duplicates
	neighbors[point2] = list(neighbor_set) # Reconvert set back into list for point2
	# Return updated neighbors dictionary
	return neighbors

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO

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
	print(x) # Include for debugging

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

# Main Code #
# Define the world as a list of points
world_points = [] # A list of x,y points (tuples) to which the Roomba can travel
wall_points = [] # A list of x,y points (tuples) through which the Roomba cannot travel
# Dictionary for neighboring points
neighbors = {} # A dictionary; keys are each point in world_points;
# values are a list of points in world_points to which the Roomba can travel directly from key (i.e., neighbors)

# Set initial direction of Roomba using the IMU
angle = imu.CalculateHeading()

# Initial Wheel Encoder values
l_counts_current, r_counts_current = Roomba.Query(43,44)

# Choose starting location as current position (x_pos, y_pos)
x_start = x_pos
y_start = y_pos
start = (x_start, y_start) # Start Location tuple
# Add the start point to world_points and create an empty neighbors entry.
world_points.append(start)
neighbors[start] = [] 

# The point in the map we were previously at
last_point = (x_start, y_start) # Initially is our starting location

# Ask the user for the coordinates of the desired goal point.
x_goal = int(input("X-coordinate of Goal: "))
y_goal = int(input("Y-coordinate of Goal: "))

# Base for a timer that logs the time it takes to move from point to point
timer_base = time.time()

while True:
	try: # Try this code until CTRL + C is pressed
		# Defining the goal point based on input from before the while loop.
		goal = (x_goal, y_goal)
		
		if goal not in world_points: # If goal is already in the world...
			neighbors[goal] = [] # Create an empty entry for goal in neighbors
			for point in world_points: # Check every point in the world to see if it is a neighbor of goal
				for wall in wall_points: # Check against every wall point
					# Find the closest distance from wall to the line segment from goal to point
					line_dist = Intersection_Finder(goal, point, wall)
					if line_dist < ROOMBA_RADIUS: # If distance is smaller than the radius of the Roomba...
						can_form_neighbors = False # goal and point are not neighbors
						break # Don't need to check any other wall_points
					# else, check for the next wall point
				if can_form_neighbors: # If no wall_points are close enough to the line segment from goal to point...
					neighbors = Create_Neighbors(goal, point, neighbors) # goal and point are neighbors
				else: # If a wall point is too close...
					can_form_neighbors = True # Reset boolean for next iteration in "for wall in wall_points"
			# Add goal to world_points after finding its neighbors
			world_points.append(goal)
		# else, goal is already part of the world, and has neighbors
		
		# Calculate the path the Roomba needs to take and get a list.
		print ('Calculating Path...')
		path = a_star(neighbors, last_point, goal)
		print ('Path Calculated: ', path, '...')
		
		# Initialize bases for timers.
		query_base = time.time() # The timer for the query.
		moveHelper = (time.time() - (break_time + spin_time + back_time)) # Timer for moving back, spinning, and breaking.
		
		# Iterate through the points to get to the goal.
		for p in path:
			print ('Moving to', p, '...')
			(m, n) = p # Get x,y components from p
			# Distance Roomba needs to travel to reach p
			desired_distance = math.sqrt(pow((n - y_pos),2) + pow((m - x_pos),2))
			# Roomba moves toward p until it is less than 2 millimeters away.
			while (desired_distance > 2):
				# Get the values and redo the calculations every 15 milliseconds.
				if (time.time() - query_base) > 0.015:
					# Roomba Data Calculations #
					l_counts, r_counts, bumpVal, lightVal = Roomba.Query(43,44,7,45)
					
					# Record the current time since the beginning of loop
					data_time = time.time() - timer_base
					
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
					# Calculated the forward distance traveled since the last counts
					distance_change = DISTANCE_CONSTANT * (delta_l_count + delta_r_count) * 0.5
					# Calculated the turn angle change since the last counts
					angle_change = TURN_CONSTANT * (delta_l_count - delta_r_count)
					angle += angle_change # Update angle of Roomba and correct for overflow
					if angle >= 360 or angle < 0:
						angle = (angle % 360) # Normalize the angle value from [0,360)
					# Calculate position data
					delta_x_pos = distance_change * math.cos(math.radians(angle))
					delta_y_pos = distance_change * math.sin(math.radians(angle))
					x_pos += delta_x_pos
					y_pos += delta_y_pos
					
					# The direction from the current position to the desired position
					desired_heading = (math.degrees(math.atan2((n - y_pos),(m - x_pos))) % 360)
					# The distance from the current position to the desired position
					desired_distance = math.sqrt(pow((n - y_pos),2) + pow((m - x_pos),2))
					
					# Update current wheel encoder counts
					l_counts_current = l_counts
					r_counts_current = r_counts
					
					# Check if a bump was detected
					if (bumpVal % 4) > 0: # If there was a bump...
						moveHelper = time.time() # Reset the timer base
						
						if (bumpVal % 4) == 2: # If it was the left bumper...
							# The wall is (approximately) 45 degrees to the left of the Roomba's current position (including the radius of the Roomba)
							x_wall = x_pos + (ROOMBA_RADIUS * math.cos(math.radians(angle - 45)))
							y_wall = y_pos + (ROOMBA_RADIUS * math.sin(math.radians(angle - 45)))
							spin = 50 # Spin right after backing up
						elif (bumpVal % 4) == 1: # If it was the right bumper...
							# The wall is (approximately) 45 degrees to the right of the Roomba's current position (including the radius of the Roomba)
							x_wall = x_pos + (ROOMBA_RADIUS * math.cos(math.radians(angle + 45)))
							y_wall = y_pos + (ROOMBA_RADIUS * math.sin(math.radians(angle + 45)))
							spin = -50 # Spin left after backing up
						else: # (bumpVal % 4) == 3: # If if was both bumpers
							# The wall is (approximately) in front of the Roomba's current position (including the radius of the Roomba)
							x_wall = x_pos + (ROOMBA_RADIUS * math.cos(math.radians(angle)))
							y_wall = y_pos + (ROOMBA_RADIUS * math.sin(math.radians(angle)))
							flip = random.randint(0, 1) # Choose 0 or 1 randomly
							spin = 50 # Set spin magnitude
							# 50% of the time, spin left
							if flip == 0:
								spin *= (-1)
							# Otherwise, spin right
						
						if can_do_order_66 == True: # If first bump this iteration of "for p in path"...
							# Force x,y values of the wall into a tuple.
							close = tuple((x_wall, y_wall))
							print ('Bump Detected At', close, '...') # Include for debugging
							wall_points.append(close) # Add the detected wall point into the list
							neighbors = Order65(last_point, p, neighbors) # There is a wall between last_point and p, so they are not neighbors
							can_do_order_66 = False # Any other bumps this iteration of "for p in path" are ignored
					
					# Movement Logic #
					# First Phase: Move Backwards
					if (time.time() - moveHelper) < (back_time):
						# Set speeds for moving backward
						move_speed = -100
						spin_value = 0
					# Second Phase: Spin Away from Wall
					elif (time.time() - moveHelper) < (back_time + spin_time):
						move_speed = 0 # No forward movement
						# Set the spin direction according to which bump was detected
						spin_value = spin
					# Third Phase: Modify World
					elif (time.time() - moveHelper) < (back_time + spin_time + break_time):
						# Stop Roomba movement
						move_speed = 0
						spin_value = 0
						
						# Remove any world_points that are too close to any wall_points
						world_points_to_remove = [] # List of world points that are too close to the new wall point, close
						for point in world_points: # Check each point in the world
							for wall in wall_points: # Check against each wall point
								if g_cost(wall, point) < ROOMBA_RADIUS: # If the distance to the new wall point is less than the radius of the Roomba...
									neighbors = Order66(point, neighbors) # Remove that point from neighbors
									world_points_to_remove.append(point) # Add it to the list of points to remove
									break # Don't need to check any more wall_points
						# Remove these points from from world_points
						for point in world_points_to_remove:
							world_points.remove(point)
						
						pp = (x_pos, y_pos) # Get Roomba's current location (after backing up and spinning)
						(x_wall, y_wall) = close # Extract x,y coordinates of the newly added wall point
						# The vector from the point to close
						vector_x = x_pos - x_wall
						vector_y = y_pos - y_wall
						vector = (vector_x, vector_y)
						# Find length and direction of vector
						zero_point = (0,0)
						vector_distance = g_cost(vector, zero_point) # millimeters
						vector_theta = math.degrees(math.atan2(vector_y, vector_x)) # degrees
						
						# Create new world points around close
						cp_list = [] # List of points to add around new wall point
						cp_list.append(pp) # Include current position of Roomba
						last_point = pp # This will be our new starting point for A* (may move further down code logically)
						
						for value in range(1,8): # Create 7 new points evenly spaced around close from pp
							rotated_angle = (vector_theta + (value * 45)) # angle to rotate vector by to create new world point
							cp_x = vector_distance * (math.cos(math.radians(rotated_angle))) + x_wall # x-coordinate of new world point
							cp_y = vector_distance * (math.sin(math.radians(rotated_angle))) + y_wall # y-coordinate of new world point
							cp_new = (cp_x, cp_y) # New world point tuple
							cp_list.append(cp_new) # Add new world point to the list
						
						print (cp_list) # Include for debugging (should be 8 points evenly spaced around close)
						# Remove any points in cp_list that are too close to any existing walls
						in_wall = False # is a given point too close to a wall?
						for point in cp_list: # Check each potential world point
							for wall in wall_points: # Check against each wall point
								if g_cost(point, wall) < ROOMBA_RADIUS: # If the distance to the wall point is less than a radius of the Roomba...
									in_wall = True # It is too close to a wall
									break # Don't need to check any other wall_points
							if in_wall: # If it is too close to a wall
								cp_list.remove(point) # Remove it from the list; it can't be reached by the Roomba
								in_wall = False # Reset boolean for the next point
						
						print (cp_list) # Include for debugging (Will only include points to which the Roomba can travel)
						# Add empty entries to neighbors for remaining points in cp_list
						for point in cp_list:
							neighbors[point] = []
						
						world_points += cp_list # Add cp_list to world_points
						# Determine the neighbors of the newly added world_points
						for point_1 in cp_list: # For each point in cp_list
							for point_2 in world_points: # See if it connects with each point in the world
								if point_1 == point_2: # If they are the same point...
									continue # Move to the next point in "for point_2 in world_points:"
								else:
									#print (point_1, point_2) # Include for debugging
									for wall in wall_points: # Check against every wall point
										# Find the closest distance from wall to the line segment from point_1 to point_2
										line_dist = Intersection_Finder(point_1, point_2, wall)
										if line_dist < ROOMBA_RADIUS: # If distance is smaller than the radius of the Roomba...
											can_form_neighbors = False # point_1 and point_2 are not neighbors
											break # Don't need to check any other wall_points
										# else, check for the next wall point
									if can_form_neighbors: # If no wall_points are close enough to the line segment from goal to point...
										neighbors = Create_Neighbors(point_1, point_2, neighbors) # point_1 and point_2 are neighbors
									else: # If a wall point is too close...
										can_form_neighbors = True # Reset boolean for next iteration in "for wall in wall_points"
						
						take_a_break = True # Used to break out of "for p in path:"
						break # while desired_distance < 2:

					# Fourth Phase: Move To Next Point (default)
					else:
						# Get the spin and move speeds for heading to p
						spin_value = DHTurn(angle, desired_heading, 0.5)
						move_speed = DDSpeed(angle, desired_heading, desired_distance)

						if lightVal > 0: # If the light bumpers detect something...
							# Cut speeds in half
							#maybe just do the move_speed, and not spin_speed?
							move_speed = move_speed // 2
							spin_value = spin_value // 2
					
					# Determine Roomba movement speeds based on set values
					# Increment move speed by move_step to avoid sudden changes in movement.
					if actual_move < move_speed:
						actual_move += move_step
						if actual_move > move_speed:
							actual_move = move_speed
					if actual_move > move_speed:
						actual_move -= move_step
						if actual_move < move_speed:
							actual_move = move_speed
					# Increment spin speed by move_step to avoid sudden changes in movement.
					if actual_spin < spin_value:
						actual_spin += move_step
						if actual_spin > spin_value:
							actual_spin = spin_value
					if actual_spin > spin_value:
						actual_spin -= move_step
						if actual_spin < spin_value:
							actual_spin = spin_value
					
					# Move at the desired speed
					Roomba.Move(actual_move, actual_spin)
					# Reset the query and do it again in 15 milliseconds to keep consistency.
					query_base += 0.015
			# End "while (desired_distance > 2):"
			
			if take_a_break == True: # If "while (desired_distance > 2):" was stopped due to a bump...
				Roomba.Move(0,0) # Stop Roomba movement
				# Reset booleans for next path
				take_a_break = False
				can_do_order_66 = True
				break # "for p in path:"
			# Otherwise, we reached p in path, so make it our lastpoint reached
			last_point = (m, n) 
			print ('Reached',last_point,'...')
		# End "for p in path:"
		
		if last_point == goal: # If we reached the current goal location...
			print ('Made it!')
			Roomba.Move(0,0) # Stop Roomba movement
			Roomba.PlaySMB()
			# Ask the user for the coordinates for a new desired goal point.
			x_goal = int(input("X-coordinate of Goal: "))
			y_goal = int(input("Y-coordinate of Goal: "))
			
			# Base for a timer that logs the time it takes to move from point to point
			timer_base = time.time()
		# Otherwise, restart the loop to recalculate the path to the current goal
		
	except KeyboardInterrupt:
		print("")
		break # Ctrl + C ends while loop

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.Move(0,0)
GPIO.output(gled, GPIO.LOW) # Turn off green LED

# Cleanup the program and print program terminated.
print ("Program Terminated: Printing Debug Data ...")

print(" World Points:")
for point in world_points:
	(m, n) = point
	print ('{},{}'.format(n,m))
print(' End of World Points ...')
print("")
print(" Wall Points:")
for point in wall_points:
	(m, n) = point
	print ('{},{}'.format(n,m))
print(' End of Wall Points ...')
print("")
print(" Neighbors Dictionary:")
for key, value in neighbors.items():
	(a,b) = key
	print ('{0},{1}'.format(b,a))
	for (m,n)  in value:
		print ('{0},{1}'.format(n,m))
	print ('')
print(' End of Neighbors Dictionary ...')
Roomba.ShutDown()
GPIO.cleanup()
