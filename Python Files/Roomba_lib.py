''' Roomba_lib.py
Purpose: Python Library with Roomba specific functions
Import this file in main Python file to access functions
See the iRobot Roomba 600 OI specifications document
	for more details on specific actuator commands.
Last Modified: 5/31/2018
'''
## Import Libraries ##
import time
import serial

## iRobot Create 2 (Roomba) Class ##
class Create_2:
	ddPin = 0 # Integer specifying ddPin placement on Raspberry Pi
		# Must be set after creating method object
	# Dictionary of packet byte size and sign
		# False = unsigned; True = signed;
	packet_dict = {
		7: [1, False], # Bump and Wheel Drops
		8: [1, False], # Wall (Light Bumper - Right)
		9: [1, False], # Cliff Left
		10: [1, False], # Cliff Front Left
		11: [1, False], # Cliff Front Right
		12: [1, False], # Cliff Right
		13: [1, False], # Virtual Wall
		14: [1, False], # Wheel Overcurrents
		15: [1, False], # Dirt Detect
		16: [1, False], # Unused Byte
		17: [1, False], # Infrared Character Omni
		18: [1, False], # Buttons
		19: [2, True], # Distance
		20: [2, True], # Angle
		21: [1, False], # Charging State
		22: [2, False], # Battery Voltage
		23: [2, True], # Battery Current
		24: [1, True], # Battery Temperature
		25: [2, False], # Battery Charge
		26: [2, False], # Battery Capacity
		27: [2, False], # Wall Signal (Light Bump Right Signal)
		28: [2, False], # Cliff Left Signal
		29: [2, False], # Cliff Front Left Signal
		30: [2, False], # Cliff Front Right Signal
		31: [2, False], # Cliff Right Signal
		32: [2, False], # Unused
		33: [1, False], # Unused
		34: [1, False], # Charging Sources Available
		35: [1, False], # OI Mode
		36: [1, False], # Song Number
		37: [1, False], # Song Playing
		38: [1, False], # Number of Stream Packets
		39: [2, True], # Requested Velocity
		40: [2, True], # Requested Radius
		41: [2, True], # Requested Right Velocity
		42: [2, True], # Requested Left Velocity
		43: [2, True], # Left Encoder Counts
		44: [2, True], # Right Encoder Counts
		45: [1, False], # Light Bumper
		46: [2, False], # Light Bump Left Signal
		47: [2, False], # Light Bump Front Left Signal
		48: [2, False], # Light Bump Center Left Signal
		49: [2, False], # Light Bump Center Right Signal
		50: [2, False], # Light Bump Front Right Signal
		51: [2, False], # Light Bump Right Signal
		52: [1, False], # Infrared Character Left
		53: [1, False], # Infrared Character Right
		54: [2, True], # Left Motor Current
		55: [2, True], # Right Motor Current
		56: [2, True], # Main Brush Motor Current
		57: [2, True], # Side Brush Motor Current
		58: [1, False], # Stasis
	}
	# Dictionary of packet group byte size and packet ID list
	packet_group_dict = {
		0: [26, list(range(7,27))],
		1: [10, list(range(7,17))],
		2: [6, list(range(17,21))],
		3: [10, list(range(21,27))],
		4: [14, list(range(27,35))],
		5: [12, list(range(35,43))],
		6: [52, list(range(7,43))],
		100: [80, list(range(7,59))],
		101: [28, list(range(43,59))],
		102: [12, list(range(46,52))],
		103: [9, list(range(54,59))]
	}
	
	''' Initialization stuff for the Create_2
		Parameters:
			port = string; Raspberry Pi port location (Ex: "/dev/ttyS0")
			baud = integer; Roomba communication baud rate (Ex: 115200) '''
	def __init__(self, port, baud):
		self.conn = serial.Serial(port, baud) # Define serial port connection
	
	## Roomba Functions ##
	''' Send a single byte command directly to the Roomba
		Only does integers in range [0, 255] '''
	def DirectWrite(self, num):
		self.conn.write((num).to_bytes(1, byteorder='big', signed=False))
	
	''' Returns the number of bytes available to read from the buffer
		Returns:
			integer; number of bytes currently in the buffer '''
	def Available(self):
		return self.conn.inWaiting()
	
	''' Returns the bytes in the buffer in the order they were received
		Parameters:
			num = integer; number of bytes to read from the buffer
		Returns:
			bytes; byte sequence of length num '''
	def DirectRead(self, num):
		return self.conn.read(num)
		# Ex: 'Roomba.DirectRead(Roomba.Available()).decode()'; or maybe 'Roomba.conn.read(Roomba.conn.inWaiting())'
	
	''' Roomba Wake-up Sequence
		Parameters:
			control = integer; Control Command
			131 = Safe Mode; 132 = Full Mode (Be ready to catch it!) '''
	def WakeUp(self, control):
		self.conn.write(b'\x07') # Restart Roomba (7)
		time.sleep(8) # wait 8 seconds before continuing
		self.conn.write(b'\x80') # START command (128)
		time.sleep(1)
		self.DirectWrite(control) # Control command
		# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
		time.sleep(0.1)
	
	''' Roomba Shut-down Sequence
		Run at end of code to completely close Create_2 connection '''
	def ShutDown(self):
		self.conn.write(b'\x80') # Send Roomba to Passive Mode (128)
		self.conn.write(b'\xae') # Stop Roomba OI (174)
		time.sleep(0.05)
		self.conn.close() # Close the Roomba serial port.
		
	''' Blinks the clean button on Roomba during startup
		Helps determine that RPi -> Roomba communication is working '''
	def BlinkCleanLight(self):
		#Syntax: [139] [LED code] [LED color] [LED Intesnity]
		# Turn on Dirt Detect light and Green Clean button
		self.conn.write(b'\x8b\x19\x00\x80') # 139, 25, 0, 128
		time.sleep(0.5)
		# Change green to red
		self.conn.write(b'\x8b\x19\xff\x80') # 139, 25, 255, 128
		time.sleep(0.5)
		# Turn off Clean button
		self.conn.write(b'\x8b\x19\xff\x00') # 139, 25, 255, 0
		time.sleep(0.05)
	
	''' Send command to Roomba to move
		Parameters:
			x = integer; common wheel speed (mm/s); x > 0 -> forward motion
			y = integer; diffential wheel speed (mm/s); y > 0 -> CW motion
		Error may result if |x| + |y| > 500. '''
	def Move(self, x, y):
		RW = x - y # Right wheel speed
		LW = x + y # Left wheel speed
		self.conn.write(b'\x91') # Send command to Roomba to set wheel speeds (145)
		self.conn.write((RW).to_bytes(2, byteorder='big', signed=True))
		self.conn.write((LW).to_bytes(2, byteorder='big', signed=True))
	
	''' Request and Return a single Roomba sensor packet
		Parameters:
			packetID = integer; ID number for Roomba sensor packet
		Returns:
			integer; Value of the requested Roomba sensor packet
		Do not request the same packet more than once every 15 ms. '''
	def QuerySingle(self, packetID):
		self.conn.write(b'\x8e') # Request single sensor packet (142)
		self.DirectWrite(packetID) # Send packet ID of requested sensor
		
		while self.conn.inWaiting() == 0:
			pass # Wait for sensor packet values to be returned
		
		byte, sign = self.packet_dict[packetID] # Get packet info
		# Return the value of the requested packet
		return int.from_bytes(self.conn.read(byte), byteorder='big', signed=sign)
	
	''' Request and Return a list of Roomba sensor packets
		Parameters:
			*args = integers; sequence of integers representing ID number of Roomba sensor packets
		Returns:
			integer list; List of values of requested packets, given in the order as requested
		Do not request the same set of packets more than once every 15 ms. '''
	def Query(self, args):
		num = len(args) # Find number of packets to request
		self.conn.write(b'\x95') # Request sensor query (149)
		self.DirectWrite(num) # Number of packets to request
		for packetID in args:
			self.DirectWrite(packetID) # Send packet ID for each sensor packet to request
		
		while self.conn.inWaiting() == 0:
			pass # Wait for sensor packet values to be returned
		
		data = [] # Create empty list
		for packetID in args:
			byte, sign = self.packet_dict[packetID] # Get packet info
			# Add the sensor value to the list
			data.append(int.from_bytes(self.conn.read(byte), byteorder='big', signed=sign))
		return data # Return the list of values
	
	''' Starts Data Stream with specified packets
		Parameters:
			*args = integers; sequence of integers representing ID number of Roomba sensor packets
		'''
	def StartQueryStream(self, *args):
		num = len(args)
		self.conn.write(b'\x94') # Start sensor data stream
		self.DirectWrite(num) # Number of packets to stream
		for packetID in args:
			self.DirectWrite(packetID) # Send packet ID for each sensor packet to request
	
	''' Reads in data from Roomba sent as a Query Stream
		The sequence of bytes is very structured
		Parameters:
			*args = integers; sequence of integers representing ID numbers of Roomba sensor packets
		Returns:
			integer list; List of values of requested packets, given in the order as requested
				Returns a zero for any packet that is requested, but not sent from the Roomba
		Do not call this function until you have started or resumed the Query Stream
		Roomba updates stream every 15 ms '''
	def ReadQueryStream(self, *args):
		if self.conn.inWaiting() > 0: # If data is in the buffer...
			value_dict = {} # Empty dictionary to store packet values
			# Read in the header byte value
			header = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)
			if header == 19: # Header value for data stream
				# Number of bytes to read (not counting checksum)
				num = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)
				check = (19 + num) # cummulative total of all byte values
				
				while num > 0: # Go until we reach the checksum
					# The first byte is the packet ID
					packetID = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)
					byte, sign = self.packet_dict[packetID] # Get packet info
					# Determine the value of the current packet
					value = int.from_bytes(self.conn.read(byte), byteorder='big', signed=sign)
					value_dict[packetID] = value # Create new entry in the dictionary
					# Calculate byte sum of the packet ID and value
					check += packetID # Add in the packet ID value
					buffer = (value).to_bytes(byte, byteorder='big', signed=sign)
					byte_value = int.from_bytes(buffer, byteorder='big', signed=False) # Convert to unsigned integer
					while byte_value > 255:
						check += (byte_value % 256) # Value of low byte
						byte_value = (byte_value // 256) # Value of high byte
					check += byte_value # Add in packet value
					
					num -= (byte + 1) # Update the number of bytes left to read
				
				# Read the check sum value
				checksum = int.from_bytes(self.conn.read(1), byteorder='big', signed=False)
				check += checksum
				if (check % 256) != 0: # If checksum isn't correct
					value_dict.clear() # Erase dictionary; it is corrupted
				
			else: # Incorrect header byte value
				x = self.conn.read(self.conn.inWaiting()) # Clear out buffer; return nothing
			# return data found
			data = [] # Create empty list
			for packetID in args:
				# Add the sensor value to the list
				data.append(value_dict.get(packetID, 0))
				# Returns a zero if a packetID is not found
			
			return data # Return the list of values
		else:
			pass # Continue; no data in the buffer
		
	''' Pause the current Query Stream
		Does not erase the last set of packets requested '''
	def PauseQueryStream(self):
		self.conn.write(b'\x96\x00') # Pause the Roomba data stream (150, 0)
	
	''' Resume the current Query Stream
		Uses the last set of packets requested '''
	def ResumeQueryStream(self):
		self.conn.write(b'\x96\x01') # Resume the Roomba data stream (150, 1)
	
	''' You are not expected to understand this. :)
		'''
	def PlaySMB(self):
		#Define SMB Theme song
		self.conn.write(b'\x8c\x00\x0b') # 140, 0, 11
		self.conn.write(b'\x4c\x08\x4c\x0c\x1e\x04\x4c\x0c\x1e\x04\x48\x08\x4c\x0c\x1e\x04\x4f\x0c\x1e\x14\x43\x0c')
		#                   76,  8, 76, 12, 30,  4, 76, 12, 30,  4, 72,  8, 76, 12, 30,  4, 79, 12, 30, 20, 67, 12
		time.sleep(0.05)
		#Play song
		self.conn.write(b'\x8d\x00') # 141, 0
		time.sleep(2) # Wait for song to play
