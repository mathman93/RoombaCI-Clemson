''' Roomba_PlaySong.py edit
Purpose: Play Donkey Kong 64 Island theme song on Roomba from a list
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 4/19/2021
'''
## Import libraries ##
import serial
import time
import RPi.GPIO as GPIO
import RoombaCI_lib

## Variables and Constants ##
# LED pin numbers
yled = 5
rled = 6
gled = 13

## Functions and Definitions ##
# finds the length of the song, then adds rests to the end to make it playble (not needed)
def Song_Size(songlist):
    while (len(songlist) % 32) != 0:
        if len(songlist) % 2 == 0:
            songlist.append(15) # note value
        else:
            songlist.append(1) # duration value
    return songlist

# creates each 16 note segment
def Song_DictCreate(songlist):
    songdict = {}
    for i in range(0,int(len(songlist) / 32)):
        songdict[i] = songlist[32 * i : 32 * (i+1)]
    songdict[i+1] = songlist[32*(i+1):] # remaining bit
    return songdict

# plas each 16 note segment by rewirting song 0 with the next song segment
def Play_Song(songdict,ts,tm,loop=False):
    while True:
        try:
            for i in songdict.keys():
                songlength = int(len(songdict[i])/2) # number of notes in song
                # Write the song
                Roomba.DirectWrite(140)
                Roomba.DirectWrite(i % 4)
                Roomba.DirectWrite(songlength)
                timetotal = Song_Write(songdict[i],ts,tm)
                # Play the song
                Roomba.DirectWrite(141)
                Roomba.DirectWrite(i % 4)
                print(songdict[i]) # Include for debugging
                time.sleep(((timetotal * ts)+1) / 64)
            if loop == False:
                break

        except KeyboardInterrupt: # if you want to end the song early
            break

# plays the song in sections of 32
def Song_Write(songlist,ts,tm):
    timetotal = 0
    for i in range(len(songlist)):
        if i % 2 == 0:
            Roomba.DirectWrite(songlist[i] + tm) 
        else:
            Roomba.DirectWrite(songlist[i] * ts)
            timetotal = timetotal + songlist[i]
    return timetotal

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

'''
Start of song setup after roomba setup

What this is trying to do is to allow any song to be played by only having a text file of it in a form that the roomba can understand (Roomba.DirectWrite values)
1. find song length
2. make devisable by 32
3. play the song in 32 value segments
4. while song is playing load the next 32 value segment
5. play untill song is over or user interupt
'''

timestep = 11 # (1/64)ths of a second
tone_mod = -2 # half step modulation of key
# Program the song onto the Roomba
rest = 15 - tone_mod # Rest note

FullSongList = [72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,84,2,rest,1,83,2,79,1,77,2,rest,4,\
                71,2,rest,1,74,2,77,1,83,2,rest,1,81,2,rest,1,80,2,rest,1,79,2,77,1,76,2,rest,4,\
                72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,88,2,rest,1,86,2,84,1,81,2,rest,4,\
                81,2,rest,1,83,2,84,1,84,2,79,1,76,2,72,1,78,2,rest,1,77,2,rest,1,76,2,rest,4]
# ^ string holding alternating tone and time values of a song (in this example its Donkey Kong 64 music)

#songlist = Song_Size(FullSongList)  #resize the song to be a mutple of 16                     | both of these are not needed
#songdict = Song_DictCreate(songlist) #creates a dictionary that holds each 16 note segment    | due to not needing to be a list of 32's
songdict = Song_DictCreate(FullSongList) #creates a dictionary that holds each 16 note segment
Play_Song(songdict,timestep,tone_mod,True) # plays the song

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
