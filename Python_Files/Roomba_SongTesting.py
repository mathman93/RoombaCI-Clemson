''' Roomba_SongTesting edit (copy of Roomba_SongAndSomething.py)
Purpose: Play a song and do somthing else, by looking at the status regrester of the roomba to tell when it is playing a song or not
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 10/4/21
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import RoombaCI_comps as comps
## Variables and Constants ##
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##

# Splits songlist into segments of 16 note-duration pairs for Roomba playback
def Song_DictCreate(songlist):
	songdict = {}
	index = 0
	n = len(songlist)
	while n > 32:
		song, songlist = Song_Split(songlist) # Get 16 note-duration pairs for song
		songdict[index] = song # Assign song to dictionary
		index += 1 # Increment counter
		n -= 32 # Update length of songlist
	# End while
	songdict[index] = songlist
	return songdict

# Separate first 16 note-duration pairs from fullsonglist
def Song_Split(fullsonglist):
	song = fullsonglist[0:32]
	remain = fullsonglist[32:]
	return [song, remain]

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
	x = Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
	#print(x) # Include for debugging
# End if Roomba.Available()
print(" ROOMBA Setup Complete")
GPIO.output(gled, GPIO.LOW) # Indicate all set sequences are complete

'''main program starts'''
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
while True:
	print("Here are the available compositions:\n")
	# Display Composition key names:
	complist = sorted(comps.Comp_dict.keys())
	disp_str = ""
	for key in complist:
		disp_str = disp_str + key + "; "
	# End for
	print(disp_str + "\n")
	compstr = input("Which composition would you like to play? ")
	if compstr in complist:
		break
	else:
		print("Composition Name not valid. Try again.")
		continue
	# End if
# End while

while True:
	print("The available parts for this composition are:\n")
	# Display Part key names:
	partlist = sorted(comps.Comp_dict[compstr].keys())
	disp_str = ""
	for key in partlist:
		disp_str = disp_str + key + "; "
	# End for
	print(disp_str + "\n")
	partstr = input("Which part would you like to play? ")
	if partstr in partlist:
		FullSongList = comps.Comp_dict[compstr][partstr]
		break
	else:
		print("Part Name not valid. Try again.")
		continue
	# End if
# End while

# Declare variables
i = 0 # Song dictionary index
is_on = False
wsp = 1 # added a var. to see if there was a song playing
j = 0 # Roomba song number
songdict = Song_DictCreate(FullSongList) # create song dictonary
Roomba.Write_Song(songdict[i],j,0) # writing the first song segment before te start of the main loop

message = '1' # Change this to any character string you want
Xbee.write(message.encode()) # Send the number over the Xbee
waitTimer = time.time()
while True: # Wait for everyone loop
	# time if statement
	if(time.time() - waitTimer) > 5:
		break
	# receive if statement
	if Xbee.inWaiting() > 0: # If there is something in the receive buffer
		message = Xbee.read(Xbee.inWaiting()).decode() # Read all data in
		#print(message) # To see what the message is
		# Reset timer
		waitTimer = time.time()
# End while loop

sn,isp = Roomba.Query(36,37) # Get initial values
Roomba.StartQueryStream(36,37) # start of query stream
timer = time.time() # start timer
# start main loop
while True:
	try:
		# playing the song segments
		if Roomba.Available() > 0:
			sn,isp = Roomba.ReadQueryStream(36,37)  # if roomba availble, update song number and is song playing
			#print(isp) # Include for debugging
			# writing the song segment
			if isp == 1 and wsp == 0:
				i = (i+1)%(len(songdict)) # update i, changed to use the number elements in the song dictonary
				j = (j+1) % 2
				Roomba.Write_Song(songdict[i],j,0) # wirtes the i'th song segment 
			# End if isp == 1
			# playing the song segment
			if isp == 0:
				Roomba.Play_Song(j) # plays the i'th song segment
				#print(songdict[i]) # Include for debugging
			# End if isp == 0
		# End if Roomba.Available
		
		# The LED will toggle on/off every 0.5 seconds 
		if (time.time() - timer) > 0.5:
			timer = time.time() # Reset timer value 
			if is_on: # If LED was on...
				GPIO.output(gled, GPIO.LOW) # Turn off green LED
				is_on = False
			else: # If LED was off...
				GPIO.output(gled, GPIO.HIGH) # Turn on green LED
				is_on = True
			# End if is_on
		# End if timer
		wsp = isp

	except KeyboardInterrupt: # if you want to end the song early
		break
	# End try
# End while
Roomba.Move(0,0) #stop roomba movement

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
