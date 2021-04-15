''' MapAndMove.py
Purpose: Draws a virtual map with the origin and the coordinates given, and moves from the start to its goal.
Also adds points that it bumps into to the map as walls	that it will attempt to move around to get to the goal, and keep the walls in memory for its movement in the future
Written by: David Croft and Sam Buckley
Last Modified: 4/1/2021
'''

## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import os.path
import math
import heapq

## Variables and Constants ##
file_create = True # Boolean to set for creation of data file
manual_input = False # Boolean to set manual input of points to explore
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##
''' Queue that allows for elements with a higher priority (lower priority value) to be put closer to the front of the line.
	Created using documentation from https://www.redblobgames.com/pathfinding/a-star/implementation.html
	'''
class PriorityQueue:
	def __init__(self): # Initialize queue with no elements
		self.elements = []
	# End __init__
	def empty(self): # Returns True if queue is empty, otherwise it returns false
		return len(self.elements) == 0
	# End empty
	def put(self, item, priority): # Put item into the queue with priority
		heapq.heappush(self.elements,(priority,item))
	# End put
	def get(self): # Takes first thing from list
		return heapq.heappop(self.elements)[1]
	# End get
# End PrioirtyQueue
''' Defines a world, finds the neighbors of certain points, and the location of thos points.
	Created using documentation from https://www.redblobgames.com/pathfinding/a-star/implementation.html
	'''
class GridWorld:
	def __init__(self):
		self.edges = {} # Points that can connect to other points in the world
		self.points = [] # Points that exist in the world
		self.walls = [] # Walls that are found in the world
	# End __init__
	# Tells you which points are able to be connected to
	# Note: ID needs to be a tuple 
	def neighbors(self,id):
		return self.edges.get(id,[])
	# End neighbors
	# Removes the point from the world at the specified tuple 'point' from the world 'MyWorld'
	def removePointFromWorld(self, point):
		neighborlist = self.edges.pop(point)
		for p in neighborlist:
			self.edges[p].remove(point)
		# End for
		self.points.remove(point)
	# End removePointFromWorld
	# Adds an edge from the world between two given tuple coordinate points
	def addEdgeToWorld(self, point1, point2):
		print("{0},{1}".format(point1,point2))
		if point1 in self.points and point2 in self.points:
			ls1 = self.edges[point1]
			ls1.append(point2)
			self.edges[point1] = ls1
			ls2 = self.edges[point2]
			ls2.append(point1)
			self.edges[point2] = ls2
		else:
			print("Point is not in world.")
		# End if
	# End addEdgeToWorld
	# Removes an edge from the world between two given tuple coordinate points
	def removeEdgeFromWorld(self, point1, point2):
		if point1 in self.points and point2 in self.points:
			ls1 = self.edges[point1]
			ls1.remove(point2)
			self.edges[point1] = ls1
			ls2 = self.edges[point2]
			ls2.remove(point1)
			self.edges[point2] = ls2
		else:
			print("Point is not in world.")
		# End if
	# End removeEdgeFromWorld
	# Inserts a point into the world and attaches edges to it from all other points as long as there is not a wall in the way
	def integrateIntoWorld(self, point):
		self.edges[point] = [] # Create an edge dictionary entry for the point
		self.points.append(point) # Add point to the world
		for p in self.points: # Check each point to see if it can connect
			point_check = True
			if p != point: # If the points aren't the same...
				for wall in self.walls:
					if CanMakeEdge(p,point,wall) == False: # If cannot make edge between the two points...
						point_check = False # Don't make an edge
						break # Go on to next point to check
					# End if CanMakeEdge
				# End for wall
				if point_check: # If can make an edge...
					self.addEdgeToWorld(p,point) # Add the edge between the points
				# End point_check
			# End if p
		# End for p
	# End integrateIntoWorld
	# Display current information about MyWorld
	def displayInfo(self):
		print("World Points: {0}".format(self.points)) # Current points in the world
		for point in self.edges.keys():
			value = self.edges[point]
			print("{0}:{1}".format(point,value)) # Current edges in the world
		# End for point
		print("World Walls: {0}".format(self.walls)) # Current walls in the world
	# End displayInfo
# End GridWorld

''' Calculates euclidian distance between two given tuple coordinate points
	'''
def distance(p1,p2):
	return math.sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)
# End distance
''' Creates a grid world with the two given "start" and "goal" locations
	'''
def makeworld(start,goal):
	# Call the class gridworld
	MyWorld = GridWorld()
	MyWorld.points.append(start)
	MyWorld.edges[start] = []
	MyWorld.points.append(goal)
	MyWorld.edges[goal] = []
	MyWorld.addEdgeToWorld(start,goal)
	return MyWorld
# End makeworld
''' Uses A* method of pathfinding to find best path the fastest between a given start and goal on the given plane "MyWorld"
	'''
def A_star(start,goal,MyWorld):
	frontier = PriorityQueue() # Frontier of points that havent been searched
	frontier.put(start,0) # Put the start point in frontier
	came_from = {} # Dictionary of points that other points came from
	came_from[start] = None # We begin at the start point
	cost_so_far = {} # Dictionary of "cost" to get to a point
	cost_so_far[start] = 0 # No cost to start
	while not frontier.empty(): # Keep going as long as there are points to search
		current = frontier.get() # Get the next point to search
		print("Searching {0}".format(current))
		if current == goal: # If the place we are at is the goal end the search
			break # Stop searching
		# End if current
		for next in MyWorld.neighbors(current): # Search for each point that is next to current
			# Cost to move to next point = current cost + cost to move to next + change in direction to next
			new_cost = cost_so_far[current] + distance(current,next) + angle_cost(came_from[current],current,next)
			# If new_cost is the lowest cost so far to get to next... (If next had not been searched previously, return +infinity)
			if new_cost < cost_so_far.get(next,math.inf): 
				cost_so_far[next] = new_cost # Update cost to move to next point
				# Add next to the priority queue
				priority = new_cost + distance(next,goal)
				frontier.put(next,priority)
				came_from[next] = current # Update best place to come from to get to next
			# End if new_cost
		# End for next
	# End while
	# Find the path from the start to end using came_from dictionary
	try:
		current = goal
		path = []
		while current != start:
			print("Current: {0}".format(current))
			path.append(current)
			current=came_from[current]
		# End while
		path.append(start)
		path.reverse()
		return path
	except KeyError:
		return None
# End A_star
''' Calculates a cost used to determine the past path in regards to how the Roomba rotates according to its previous, current, and next locations in its path
	'''
def angle_cost(previous,current,next):
	if previous == None: # If first movement...
		return 0 # Nothing to compare angle to
	else:
		theta = math.atan2(current[1]-previous[1],current[0]-previous[0]) # Finds current rotation
		theta_initial = math.atan2(next[1]-current[1],next[0]-current[0]) # Finds angle of rotation towards next point
		theta_d = theta_initial - theta # Finds difference between the two angles
		# Normalizes difference between -pi and pi
		if theta_d > math.pi:
			theta_d -= 2*math.pi
		elif theta_d <= -math.pi:
			theta_d += 2*math.pi
		# End if theta_d
		return abs(theta_d)
	# End if previous
# End angle_cost
''' Finds if it is possible for a path to be formed without intersecting a circle drawn around a given wall tuple coordinate
	'''
def CanMakeEdge(start,goal,wall):
	#Gets values for vectors from start to goal and from start to wall
	ax = goal[0] - start[0]
	ay = goal[1] - start[1]
	bx = wall[0] - start[0]
	by = wall[1] - start[1]
	# Normalizes both vectors
	norm_a = math.sqrt((ax**2)+(ay**2))
	dot_ab = (ax*bx) + (ay*by)
	b1 = dot_ab / norm_a
	norm_b = math.sqrt((bx**2)+(by**2))
	# Checks if line can be drawn from start to goal and won't collide with wall
	if b1 >= norm_a or b1 <= 0:
		return True
	elif (norm_b**2) - (b1**2) > (200**2):
		return True
	else:
		return False
	# End if
# End CanMakeEdge
''' Function that returns the angle of an object (in degrees in the range -70 to +70) that the Roomba is bumping into.
	Uses the 'bumper' bumper reading (query code 7) and the 'l_bumper' light bumper reading (query code 45), returns angles in range of -70,-45,-20,0,20,45,70
	'''
def BumpAngle(bumper,l_bumper):
	l_bumper_list = []
	for i in range(6): # For all the possible binary digits that represent light bumpers being activated...
		if l_bumper & pow(2,i) == pow(2,i): # If the light bumper value indicates that the "i" light bumper is being triggered...
			l_bumper_list.append(True) # The placeholder boolean for that light bumper is set to true
		else: # If the "i" light bumper is not being triggered...
			l_bumper_list.append(False) # The placeholder boolean is set to false
		# End if
	# End for i
	print(l_bumper_list) # Include for debugging
	[L,FL,CL,CR,FR,R] = l_bumper_list # Sets the booleans to be used in conditionals to their own variables for easy reference
	if bumper == 3: # If roomba detects a bump in the center...
		return 0 # Bump occured directly ahead
	# End if bumper == 3
	if bumper == 1: # If the roomba detects a bump on the right...
		if not (CL or CR or FR): # If the center two or front right light bumpers are not triggered at all...
			return math.radians(70) # Extreme right bumper angle
		elif CL: # If the center left light bumper is triggered...
			return math.radians(20) # Slight right bumper angle
		else: # Otherwise, can't determine precise angle
			return math.radians(45) # Average right bumper angle
		# End if
	# End if bumper == 1
	if bumper == 2: # If roomba detects a bump on the left...
		if not (L or FL or CL or CR): # If any of the left side or center right light bumpers are not triggered...
			return math.radians(-70) # Extreme left bumper angle
		elif CR: # If the center right light bumper is triggered...
			return math.radians(-20) # Slight left bumper angle
		else: # Otherwise, can't determine precise angle
			return math.radians(-45) # Average left bumper angle
		# End if
	# End if bumper == 2
	# Note, bumper == 0 is never True when BumpAngle is called
# End BumpAngle

def NextGoal(x,y,dx,dy):
	if x == y or (x < 0 and x == -y) or (x > 0 and x == dxy-y): # each time it gets to a corner in the spiral switch the increment variables
		dx, dy = -dy, dx
		corner_increment = 1
	else:
		corner_increment = 0
	x += dx
	y += dy
	return x,y,dx,dy,corner_increment
# End NextGoal
## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
RoombaCI_lib.DisplayDateTime() # Display current date and time
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
	z = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	#print(z) # Include for debugging
# End if Roomba.Available
print(" ROOMBA Setup Complete")
GPIO.output(gled, GPIO.LOW)

if file_create == True:	# Open a text file for data retrieval
	file_name_input = input("Name for data file: ") # Ask user for desired file name
	dir_path = "/home/pi/RoombaCI-Clemson/Data_Files/2021_Spring/" # Directory path to save file
	file_name1 = os.path.join(dir_path, file_name_input+"_points.txt") # text file extension
	file_name2 = os.path.join(dir_path, file_name_input+"_walls.txt") # text file extension
	file1 = open(file_name1, "w") # Open a text file for storing roomba points
	file2 = open(file_name2, "w") # Open a text file for storing wall points
		# Will overwrite anything that was in the text file previously
# End if file_create

#start_time = time.time() # Not sure what this is for...
# Retrieve and set initial wheel encoder values
[left_encoder, right_encoder] = Roomba.Query(43,44)
Roomba.SetWheelEncoderCounts(left_encoder,right_encoder)

# Variables and Constants
backup_time = 1.0 # Amount of time spent backing up
corner_time = 1.5 # Amount of time that it takes before the Roomba starts turning more sharply (makes sure it turns around corners)
#f = 0 # Forward/Backward speed
#s = 0 # Rotation Speed
bump_count = 0 # Keeps track of how many times the Roomba has bumped into a wall
bump_mode = False # Used to tell whether or not the Roomba has bumped into something and is supposed to be "tracking"
#bump_code = 0 # Used to distinguish if the right, left, or center bumpers are being triggered
dxy = 1000 # determines the distance between eacch point in the spiral
dx = dxy # change in x variable
dy = 0 # change in y variable
corner = 0

if manual_input == True:
	while True: #Loop that asks for initial x and y coordinates
		try:
			print("Where would you like to go?")
			x_final = int(input("X axis coordinate: "))
			y_final = int(input("Y axis coordinate: "))
			break
		except ValueError:
			print("Not a valid input. Please enter an integer.")
			continue
		# End try
	# End while
else:
	# first automated point will always be the same
	x_final = dx
	y_final = dy
# End if manual_input

start = (0,0) # Starting position in the MyWorld grid
goal = (x_final,y_final) # Final goal
MyWorld = makeworld(start,goal) # Creates a grid world for the roomba to move in with two points, the start and goal, and draws a line between them
path = A_star(start,goal,MyWorld) # Creates the optimal pathway between the start and goal
current_point = start # Saves grid coordinate that the roomba just came from
bump_break = False # Checks if the roomba has bumped into something and broken out of the loop
goal_wall_break = False # Checks if the goal point is unreachable because it is too close to a wall

#Print Stuff
print(path) # Include for debugging
MyWorld.displayInfo()

bump_time = time.time() - 2.0 # Assures that the roomba doesn't start in backup mode
data_start = time.time()
while True: # Main code execution loop
	try:
		for point in path:
			current_goal = point # Head to the next point in the path
			distance_to_end = math.sqrt((current_goal[0]-Roomba.X_position)**2 +(current_goal[1]-Roomba.Y_position)**2) # Distance of straight line between where the Roomba is and where the end point is
			theta_initial = math.atan2((current_goal[1]-Roomba.Y_position),(current_goal[0]-Roomba.X_position)) # Angle of the line between the x-axis and the initial distance to end line
			theta_d = theta_initial - Roomba.heading # Rotation needed from current heading to face goal
			#print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6:.6f},{7:.6f}"\
			#	.format(time.time()-data_start,left_start,right_start,Roomba.X_position,Roomba.Y_position,theta,distance_to_end,theta_d))
			Roomba.StartQueryStream(7,43,44,45) # Start getting wheel encoder & bumper values
			while distance_to_end > 3: # Begin heading to the next point in path
				if Roomba.Available() > 0:
					data_time = time.time()-data_start
					# Get bump value, then get left and right encoder values and find the change in each
					[bump, left_encoder, right_encoder, l_bump] = Roomba.ReadQueryStream(7,43,44,45)
					# Update X,Y Position of Roomba
					Roomba.UpdatePosition(left_encoder, right_encoder)
					x_pos_int = int(Roomba.X_position)
					y_pos_int = int(Roomba.Y_position)
					# Find distance to end and theta_initial
					distance_to_end = math.sqrt((current_goal[0]-Roomba.X_position)**2 +(current_goal[1]-Roomba.Y_position)**2)
					theta_initial = math.atan2((current_goal[1]-Roomba.Y_position),(current_goal[0]-Roomba.X_position))
					# Normalize what theta initial is to between 0-2pi
					#if theta_initial < 0: # Might not need this; can just use "if theta_d > math.pi"
					#	theta_initial += 2*math.pi
					# End if theta_initial
					# Calculate theta_d and normalize it to range [-pi, pi]
					# This value is the difference between the direction were supposed to be going and the direction we are going
					theta_d = ((theta_initial-Roomba.heading)%(2*math.pi))
					if theta_d > math.pi: # get theta_d between -pi and pi
						theta_d -= 2*math.pi
					# End if theta_d
					# Checks if the Roomba has bumped into something, and if so, activates wall detection protocol
					if (bump%4) > 0: # If the Roomba bumps into something...
						bump_count += 1 # Updates the amount of times the bumpers have detected a bump
						wall_dir = BumpAngle(bump,l_bump) # Determine estimate of direction to wall using what the light bumper detected when the Roomba bumped
						print('Bump Angle: {0:.4f}'.format(wall_dir))
						if bump_count == 1:# If first bump in the cycle...
							MyWorld.removeEdgeFromWorld(current_point,current_goal) # Remove the edge currently travelling on (there's a wall in the way)
							x_wall = int(x_pos_int + (200*math.cos(Roomba.heading + wall_dir))) # Calculates x position of wall
							y_wall = int(y_pos_int + (200*math.sin(Roomba.heading + wall_dir))) # Calculates y position of wall
							MyWorld.walls.append((x_wall,y_wall)) # Adds the coordinate position of the wall to the list of walls
							print("New Wall Made: {0}".format((x_wall,y_wall)))
							# Removes all points too close to new wall
							points_to_remove = []
							for p_r in MyWorld.points:
								if distance(p_r,(x_wall,y_wall)) < 200: # If the point is withing 200 mm to wall point...
									points_to_remove.append(p_r) # Add point to list of points to remove
								# End if distance
							# End for p_r
							for p in points_to_remove:
								MyWorld.removePointFromWorld(p)
								if p == goal: # If the goal point is close enough to the new wall to be removed...
									print("Goal Point is Inaccessible, Too Close to Wall")
									goal_wall_break = True # Verifies that the goal point has been removed
								# End if p
							# End for p
						# End if bump_count == 1
						bump_time = time.time() # Sets up timer that tells how long to back up
					# End if bump%4
					if time.time() - bump_time < 1.0: # If Roomba has bumped into something less than 1 second ago...
						# Have Roomba move straight backward
						f = -100
						s = 0
					elif time.time() - bump_time < 1.5: # If done backing up...
						new_points = [None,None,None] # Initialize list of new World points to be added
						current_point = (x_pos_int,y_pos_int) # Get current position of Roomba
						bump_break = True # Validates that the Roomba has broken out of the loop
						new_points[0] = (x_pos_int,y_pos_int) # Current point after backing up from wall
						np1x = int(x_pos_int + (400 * math.cos(Roomba.heading + wall_dir + (math.pi/2)))) # X position of point to right of roomba
						np1y = int(y_pos_int + (400 * math.sin(Roomba.heading + wall_dir + (math.pi/2)))) # Y position of point to right of roomba
						new_points[1] = (np1x,np1y)
						np2x = int(x_pos_int + (400 * math.cos(Roomba.heading + wall_dir - (math.pi/2)))) # X position of point to left of roomba
						np2y = int(y_pos_int + (400 * math.sin(Roomba.heading + wall_dir - (math.pi/2)))) # Y position of point to left of roomba
						new_points[2] = (np2x,np2y)
						break # while distance_to_end
					else: # If haven't bumped into anything yet...
						if abs(theta_d) > (math.pi / 4): #If theta_d is greater than pi/4 radians...
							s_set = 100 # Spin faster
						elif abs(theta_d) > (math.pi / 36): #If theta_d is getting closer...
							s_set = 60 # Spin normal speed
						else: # otherwise, if theta_d is fairly small
							s_set = 20 # Spin slow
						# End if abs(theta_d)
						if distance_to_end > 150: #If distance_to_end is greater than 150 mm...
							f_set = 120 # Go faster
						elif distance_to_end > 50: # If distance_to_end is greater than 50 mm...
							f_set = 80 # Go fast
						else: #otherwise, if distance_to_end is less than 50 mm...
							f_set = 40 # Go slow  
						# End if distance_to_end   

						radius = ((235 / 2) * (f_set / s_set)) # Radius of circle of the Roomba's turn for the given f_set and s_set values

						if theta_d > 0: #Rotates clockwise if theta_d is positive
							s = s_set
						elif theta_d < 0: #Rotates counterclockwise if theta_d is negative
							s = s_set * -1
						else:
							s = 0
						# End if theta_d
						if theta_d > (math.pi / 3) or theta_d < (math.pi / -3): #If the end point is beyond 90 degrees in either direction, the roomba will rotate in place
							f = 0
						elif abs(2*radius*math.sin(theta_d)) > distance_to_end: #If the end point is within the circle that is drawn by the roomba's turn path, then the roomba will rotate in place 
							f = 0
						else:
							f = f_set
						# End if theta_d
					# End if time.time()
					Roomba.Move(f,s) # Move with given forward and spin values
					#print("{0:.6f},{1},{2},{3:.3f},{4:.3f},{5:.6f},{6:.6f},{7:.6f}, bump_count:{8}"\
					#	.format(data_time,left_encoder,right_encoder,Roomba.X_position,Roomba.Y_position,Roomba.heading,distance_to_end,theta_d,bump_count))
				# End if Roomba.Available()
			# End while distance_to_end
			Roomba.Move(0,0) # Stop Roomba movement (momentarily)
			Roomba.PauseQueryStream() #Pauses the query stream while new coordinates are being input
			if Roomba.Available() > 0:
				z = Roomba.DirectRead(Roomba.Available())
				#print(z) # Include for debugging
			# End if Roomba.Available
			if bump_break == True: # If had to break out of the loop after bumping...
				break # Stop following the current path
			# End if bump_break
			current_point = point # Roomba is now at point
			# Continue for loop to go to next point in path
		# End for point in path
		if bump_break: # If the Roomba has bumped into something and broken out of the loop...
			# Reset variables responsible for bumping operations
			bump_break = False
			bump_time = time.time() - 2.0
			bump_count = 0 # Rest bump_break value
			new_list = [] # List of points viable for the Roomba to move to after bumping into an object
			for p1 in new_points: # Check if the current point, point to the left, or point to the right are not too close to another point or too close to a current wall
				point_check = True
				for p2 in MyWorld.points:
					if distance(p1,p2) < 10: # If point is within 10mm of another point...
						point_check = False # Don't add it
						break # Stop checking other points
					# End if distance
				# End for p2
				for wall in MyWorld.walls:
					if distance(p1,wall) < 200: # If point is within 200mm of a wall...
						point_check = False # Don't add it
						break # Stop checking other points
					# End if distance
				# end for wall
				if point_check == True: # If point is fine to place...
					new_list.append(p1) # Add point to list of points to add to world
				# End if point_check
			# End for p1
			# Display current information about MyWorld
			print("new_list: {0}".format(new_list)) # Include for debugging
			for point in new_list: # For every point to be added to the world, adds to world and adds all edges possible to it from other points
				MyWorld.integrateIntoWorld(point)
			# End for point
			MyWorld.displayInfo()
			if goal_wall_break == True: # If the goal was too close to a wall to be reached...
				goal_wall_break = False
				goal = current_point # Will immediately be at end of path, and will give new coordinate prompt
			# End if goal_wall_break
			while True: # assuming that the manual input will not be put in a location which 
				path = A_star(current_point,goal,MyWorld) # Generate a new path with updated walls, points, and edges
				if path == None: # if path cannot be reached skip the point and go to the next one. 
					x_final,y_final,dx,dy,corner_increment = NextGoal(x_final,y_final,dx,dy)
					goal = (x_final,y_final) # loops back and tries again
					corner += corner_increment
				else:
					break
				# End if path
			# End while True
		else: # Roomba finished getting to goal successfully
			MyWorld.displayInfo() # Display current information about MyWorld
			start = current_point # Set new start point for path to goal
			while True: # Ask for next x,y coordinates
				try:
					if manual_input == True:
						print("Where would you like to go next?")
						x_final = int(input("X axis coordinate: "))
						y_final = int(input("Y axis coordinate: "))
					else: # creates a spiral of points with distance dxy
						x_final,y_final,dx,dy,corner_increment = NextGoal(x_final,y_final,dx,dy)
					#end if manual_input
					goal = (x_final,y_final)
					if goal not in MyWorld.points: # If the goal does not already exist in the world...
						goal_check = True
						for wall in MyWorld.walls:
							if distance(goal,wall) < 200: # If the goal is too close to an existing wall...
								print("Too close to a wall")
								goal_check = False # Don't add it to the world
								break
							# End if distance
						# End for wall
						if goal_check == True: # If the goal can be placed...
							MyWorld.integrateIntoWorld(goal) # Add it to the world
							if manual_input == False and corner_increment == 1: 
								corner = 0 # reset corner variable if you can get to a point that is a corner
							# End if manual_input
							# Go to find a path to the goal
						else: # If goal cannot be placed
							print("An error occured. Goal point cannot be reached.")
							if manual_input == False:
								corner += corner_increment # increment variable to determine when to break out of the loop
								if corner == 4: # if you reach 4 corners in a row break out of loop
									EndProgram = True # boolean variable to break out of loop
									break
								# End if corner
							continue # Ask for a new point
						# End if goal_check
					# End if goal
					break
				except ValueError:
					print("Please input a number.")
					continue
				# End try
			# End while True
			if EndProgram == True:
				break
			MyWorld.displayInfo() # Display current information about MyWorld
			while True:	
				path = A_star(start,goal,MyWorld) # Find new path to the goal with new coordinate information
				if path == None:
					x_final,y_final,dx,dy,corner_increment = NextGoal(x_final,y_final,dx,dy)
					goal = (x_final,y_final)
					corner += corner_increment
				else:
					break
				# End if path
			# End While True
			print(path) # Include for debugging
			# Go to new goal
		# End if bump_break
	except KeyboardInterrupt: # We can manually stop the Roomba at any time.
		break
	# End try
# End while
Roomba.Move(0,0) # Stop Roomba movement
Roomba.PauseQueryStream()
if Roomba.Available() > 0:
	z = Roomba.DirectRead(Roomba.Available())
	#print(z) # Include for debugging
# End if Roomba.Available
if file_create == True:	
	for p in MyWorld.points:
		file1.write("{0},{1}\n".format(p[0],p[1]))
	# End for p
	for p in MyWorld.walls:
		file2.write("{0},{1}\n".format(p[0],p[1]))
	# End for p
# End if file_create
time.sleep(0.1)
if file_create == True:
	file1.close() # Close data file
	file2.close() # Close data file
# End if file_create
## -- Ending Code Starts Here -- ##
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program