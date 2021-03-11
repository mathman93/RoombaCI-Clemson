''' Roomba_DK64.py edit
Purpose: Play Donkey Kong 64 Island theme song on Roomba as a list of a song
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/29/2018
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO

import RoombaCI_lib

## Variables and Constants ##

# LED pin numbers
gled = 13

## Functions and Definitions ##


## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO

# LED Pin setup
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

print(" ROOMBA Setup Complete")

'''
What this is trying to do is to allow any song to be played by only having a text file of it in a form that the roomba can understand (Roomba.DirectWrite values)
1. find song length
2. make devisable by 32
3. play the song in 32 value segments
4. while song is playing load the next 32 value segment
5. play untill song is over or user interupt
'''


dk64list=  [72,2,30,1,74,2,79,1,81,2,30,1,79,2,30,1,84,2,30,1,83,2,71,2,30,1,74,2,77,1,83,2,30,1,81,2,30,1,80,2,30,1,79,2,77,1,76,2,72,2,30,1,74,2,79,1,81,2,30,1,79,2,30,1,88,2,30,1,86,2,84,1,81,2]
timestep = 11 # (1/64)ths of a second
rest = 30 # Rest note
tone_mod = -2 # half step modulation of key
# Program the song onto the Roomba

# finds the length of the song, then adds rests to the end to make it playble
def Song_Size(songlist):
    while (len(songlist) % 32) != 0:
        songlist.append(30)

# plays the song in sections of 32
def Song_Write(songlist):
    for i in range(32):
        if i % 2 == 0:
            Rommba.DirectWrite(songlist(i) + tone_mod) 
        else:
            Roomba.DirectWrite(songlist(i) * timestep)


# resize song
Song_Size(dk64list)

# loop plays song
for x in range (len(dk64list)/32):
    Roomba.DirectWrite(140) # Header Opcode
    Roomba.DirectWrite(0)   # Song number (0-3)
    Roomba.DirectWrite(16)  # Length of song (0-15)
    Song_Write(dk64list)
    time.sleep(0.05) # Wait before continuing










# Define Part 4 (13 notes)
Roomba.DirectWrite(140) # Header Opcode
Roomba.DirectWrite(3)   # Song number (0-3)
Roomba.DirectWrite(13)  # Length of song (0-15)
Roomba.DirectWrite(81 + tone_mod)  # Note 1
Roomba.DirectWrite(2 * timestep)	# Note 1 duration (in 1/64 of a second)
Roomba.DirectWrite(rest)
Roomba.DirectWrite(1 * timestep)
Roomba.DirectWrite(83 + tone_mod)
Roomba.DirectWrite(2 * timestep)
Roomba.DirectWrite(84 + tone_mod)
Roomba.DirectWrite(1 * timestep)
Roomba.DirectWrite(84 + tone_mod)
Roomba.DirectWrite(2 * timestep)
Roomba.DirectWrite(79 + tone_mod)
Roomba.DirectWrite(1 * timestep)
Roomba.DirectWrite(76 + tone_mod)
Roomba.DirectWrite(2 * timestep)
Roomba.DirectWrite(72 + tone_mod)
Roomba.DirectWrite(1 * timestep)
Roomba.DirectWrite(78 + tone_mod)
Roomba.DirectWrite(2 * timestep)
Roomba.DirectWrite(rest)
Roomba.DirectWrite(1 * timestep)
Roomba.DirectWrite(77 + tone_mod)
Roomba.DirectWrite(2 * timestep)
Roomba.DirectWrite(rest)
Roomba.DirectWrite(1 * timestep)
Roomba.DirectWrite(76 + tone_mod)
Roomba.DirectWrite(2 * timestep)
# Song length = 24 * timestep

time.sleep(0.05) # Wait before continuing

song_list = [0, 1, 2, 3]
while True:
	try:
		for song in song_list:
			# Play DK64 Theme
			Roomba.DirectWrite(141) # Header Opcode
			Roomba.DirectWrite(song)   # Song number (0-3)
			
			time.sleep(24 * (timestep / 64)) # Wait for the song to play
		
	except KeyboardInterrupt:
		break

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
GPIO.output(gled, GPIO.LOW) # Turn off green LED

Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program