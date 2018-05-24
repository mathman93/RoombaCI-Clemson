''' Roomba_lib.py
Purpose: Python Library with Roomba specific functions
Import this file in main Python file to access functions
Last Modified: 5/23/2018
'''

## Roomba Functions ##
''' Converts integers into bytes (Ints To Bytes)
	Only does integers in range [0, 255]'''
def itb(num):
	return (num).to_bytes(1, byteorder='big', signed=False)

''' Roomba Wake-up Sequence
	Parameters: control = Control Command
		# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!) '''
def WakeUp(control)
	print(" Starting ROOMBA... ")
	Roomba.write(itb(7)) # Restart Roomba
	time.sleep(8) # wait 8 seconds before continuing
	Roomba.write(itb(128)) # START command
	time.sleep(1)
	Roomba.write(itb(control)) # Control command
	# 131 = Safe Mode; 132 = Full Mode (Be ready to catch it!)
	time.sleep(0.1)
	
''' Blinks the clean button on Roomba during startup
	Helps determine that RPi -> Roomba communication is working'''
def BlinkCleanLight():
	#Syntax: [139] [LED code] [LED color] [LED Intesnity]
	# Turn on Dirt Detect light and Green Clean button
	Roomba.write(itb(139))
	Roomba.write(itb(25))
	Roomba.write(itb(0))
	Roomba.write(itb(128))
	time.sleep(0.5)
	# Change green to red
	Roomba.write(itb(139))
	Roomba.write(itb(25))
	Roomba.write(itb(255))
	Roomba.write(itb(128))
	time.sleep(0.5)
	# Turn off Clean button
	Roomba.write(itb(139))
	Roomba.write(itb(25))
	Roomba.write(itb(255))
	Roomba.write(itb(0))
	time.sleep(0.05)

''' Send command to Roomba to move
	x is common wheel speed (mm/s); y is diffential wheel speed (mm/s)
	x > 0 -> forward motion; y > 0 -> CW motion
	Error may result if |x| + |y| > 500.'''
def Move(x,y):
	RW = x - y # Right wheel speed
	LW = x + y # Left wheel speed
	Roomba.write(itb(145)) # Send command to Roomba to set wheel speeds
	Roomba.write((RW).to_bytes(2, byteorder='big', signed=True))
	Roomba.write((LW).to_bytes(2, byteorder='big', signed=True))

''' You are not expected to understand this. :)
	'''
def PlaySMB():
	#Define SMB Theme song
	Roomba.write(itb(140))
	Roomba.write(itb(0))
	Roomba.write(itb(11))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(8))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(8))
	Roomba.write(itb(72))
	Roomba.write(itb(8))
	Roomba.write(itb(76))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(8))
	Roomba.write(itb(79))
	Roomba.write(itb(8))
	Roomba.write(itb(30))
	Roomba.write(itb(24))
	Roomba.write(itb(67))
	Roomba.write(itb(8))
	
	time.sleep(0.05)
	#Play song
	Roomba.write(itb(141))
	Roomba.write(itb(0))
	time.sleep(2) # Wait for song to play