''' Roomba_SongAndSomething edit (coppy of Roomba_PlaySong.py)
Purpose: Play a song and do somthing else, by looking at the status regrester of the roomba to tell when it is playing a song or not
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 10/4/21
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

# creates each 16 note segment
def Song_DictCreate(songlist):
    songdict = {}
    for i in range(0,int(len(songlist) / 32)):
        songdict[i] = songlist[32 * i : 32 * (i+1)]
    songdict[i+1] = songlist[32*(i+1):] # remaining bit
    return songdict


def Play_Song(songdict,ts,tm,i,loop=False):
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
    print(timetotal)  #how long the roomba should wait



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

program objectives
1. play the currently programed song
2. while roomba is playing the current song the roomba should be blinking the green LED
3. later this will be changed out for movement of the roomba while playing a song
'''

'''main program starts'''

timestep = 8 # (1/64)ths of a second
tone_mod = -7 # half step modulation of key
rest = 15 - tone_mod # Rest note
FullSongList = [72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,84,2,rest,1,83,2,79,1,77,2,rest,4,\
                71,2,rest,1,74,2,77,1,83,2,rest,1,81,2,rest,1,80,2,rest,1,79,2,77,1,76,2,rest,4,\
                72,2,rest,1,74,2,79,1,81,2,rest,1,79,2,rest,1,88,2,rest,1,86,2,84,1,81,2,rest,4,\
                81,2,rest,1,83,2,84,1,84,2,79,1,76,2,72,1,78,2,rest,1,77,2,rest,1,76,2,rest,4]

FullSongList = [72,1,74,1,77,1,74,1,81,3,81,3,79,6,72,1,74,1,77,1,74,1,79,3,79,3,77,2,76,1,74,3,\
                74,1,74,1,77,1,74,1,77,4,79,2,76,3,74,1,72,4,72,2,79,4,77,6,rest,2,\
                72,1,74,1,77,1,74,1,81,3,81,3,79,6,72,1,74,1,77,1,74,1,84,4,76,2,77,3,76,1,74,2,\
                74,1,74,1,77,1,74,1,77,4,79,2,76,3,74,1,72,4,72,2,79,4,77,7,rest,1] # A rick roll

# declare vars.
i = 0
ison = False

timer = time.time() # start timer
songdict = Song_DictCreate(FullSongList) # create song dictonary
sn,isp = Roomba.Query(36,37)  # get values of song number(sn) and is song playing(isp)
print(sn)
print(isp) # include for debugging
Roomba.StartQueryStream(36,37)  # start of query stream
#sn,isp = Roomba.ReadQueryStream(36,37)  # get values of song number(sn) and is song playing(isp)
# start main loop
while True:
    try:
        if Roomba.Available() > 0:
            sn,isp = Roomba.ReadQueryStream(36,37)  # if roomba availble, update song number and is song playing
            print(sn)
            print(isp) # Include for debugging
    
        if isp == 0:
                Play_Song(songdict,timestep,tone_mod,i,True) # plays the i'th song segment
                i = (i+1)%4 # update i
    
        if(time.time() - timer > 0.5):
            timer = time.time() # using a timer, every 0.5 seconds a LED will toggle on/off
            if ison == False:
                GPIO.output(gled, GPIO.HIGH) # Turn on green LED
                ison = True
            
            if ison == True:
                GPIO.output(gled, GPIO.LOW) # Turn off green LED
                ison = False

    except KeyboardInterrupt: # if you want to end the song early
        break
    # End while


## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program