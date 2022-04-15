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

# creates each 16 note segment
def Song_DictCreate(songlist):
    songdict = {}
    for i in range(0,int(len(songlist) / 32)):
        songdict[i] = songlist[32 * i : 32 * (i+1)]
    songdict[i+1] = songlist[32*(i+1):] # remaining bit
    return songdict

def Play_Song(j):
    Roomba.DirectWrite(141)
    Roomba.DirectWrite(j)

# plays the song in sections of 32
def Song_Write(songlist,tm,j):
    songlength = int(len(songlist)/2) # number of notes in song
    Roomba.DirectWrite(140)
    Roomba.DirectWrite(j)
    Roomba.DirectWrite(songlength)
    timetotal = 0
    for i in range(len(songlist)):
        if i % 2 == 0:
            Roomba.DirectWrite(comps.Note_dict[songlist[i]] + tm) 
        else:
            Roomba.DirectWrite(songlist[i])
            timetotal = timetotal + songlist[i]
    return timetotal

def Movement_Sync_list(songlist,ts,rest):
    t_list = []
    t = 0
    for i in range(len(songlist)):
       if (i % 2 == 0):
           if comps.Note_dict[songlist[i]] == 30:
               t_list.append(t * ts)
               t = 0
       else:
            t = t + songlist[i]
    print(t_list)
    return t_list


def NoteReturn(note):
    list = comps.Note_dict.keys()
    if note in list:
        return (comps.Note_dict[note])
    #else:
    #    print("Note not valid")
    #    note = input("Enter a valid note G1-Ab5")
    #    NoteReturn(note)
def SongSelect():
    while True:
        list = comps.Comp_dict.keys()
        for key in list:
            print(key)
            print(" ")
        compstr = input("Which song would you like to play? ")
        if compstr in list:
            for key in comps.Comp_dict[compstr].keys():
                print(key)
            partstr = input("Which part would you like to play? ")
            list = comps.Comp_dict[compstr].keys()
            if partstr in list:
                FullSongList = PartReturn(compstr,partstr)
                return FullSongList
        else:
            print("Song Name not Valid")
            continue
             

def PartReturn(compstr,partstr):
    return comps.Comp_dict[compstr][partstr]

    
## -- Code Starts Here -- ##
global Xbee # Specifies connection to Xbee
Xbee = serial.Serial('/dev/ttyUSB0', 115200) # Baud rate should be 115200
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
rest = 15 - tone_mod #Rest note
#while True:
#clean up the user interface
# added function to select song, needs some work 
FullSongList = SongSelect()
'''
    compstr = input("Which song would you like to play? Enter DK for Donkey Kong or RickRoll for Rick Roll ")
    list = comps.Comp_dict.keys()
    if compstr in list:
        for key in comps.Comp_dict[compstr].keys():
            print(key)
            print(" ")
        partstr = input("Which part would you like to play? ")
        list = comps.Comp_dict[compstr].keys()
        if partstr in list:
            FullSongList = comps.Comp_dict[compstr][partstr]
            break
    else:
        print("Song Name not Valid")
        continue
'''
# declare vars.
i = 0
y = 0
is_on = False
spin = False
wsp = 1 # added a var. to see if there was a song playing
timer = time.time() # start timer
waitTimer = time.time() #start a second timer for movement
t_list = Movement_Sync_list(FullSongList,timestep,rest)
j = 0 # song position, i is song dictonary position
songdict = Song_DictCreate(FullSongList) # create song dictonary
sn,isp = Roomba.Query(36,37)
Roomba.StartQueryStream(36,37)  # start of query stream
Song_Write(songdict[i],tone_mod,i) # writing the first song segment before te start of the main loop

message = '1' # Change this to any character string you want
Xbee.write(message.encode()) # Send the number over the Xbee
while True: # Wait for everyone loop
	# time if statement
	if(time.time()-timer)>5:
		break
	# receive if statement
	if Xbee.inWaiting() > 0: # If there is something in the receive buffer
		message = Xbee.read(Xbee.inWaiting()).decode() # Read all data in
		#print(message) # To see what the message is
		# Reset timer
		timer = time.time()
# End while loop

# start main loop
while True:
    try:
        # playing the song segments
        if Roomba.Available() > 0:
            sn,isp = Roomba.ReadQueryStream(36,37)  # if roomba availble, update song number and is song playing
            
            # writing the song segment
            if isp == 1 and wsp == 0:
                i = (i+1)%(len(songdict)) # update i, changed to use the number elements in the song dictonary
                j = (j+1) % 2
                Song_Write(songdict[i],tone_mod,j) # wirtes the i'th song segment 

            # playing the song segment
            if isp == 0:
                Play_Song(j) # plays the i'th song segment
                print(songdict[i]) # Include for debugging

            # moving the Roomba, needs to be under Roomba.Available (update to sync with song)
            if (time.time() - timer2) > (0.015625 * t_list[y]) :
                timer2 = time.time() 
                Roomba.Move(0,0) #stop roomba movement
                if spin:
                    Roomba.Move(0,0) # spin clockwise
                    spin = False
                else:
                    Roomba.Move(0,0) # spin counterclockwise
                    spin = True
                y = y + 1
                if y == len(t_list):
                   y = 0

        # blinking the LED
        if (time.time() - timer) > 0.5:
            timer = time.time() # using a timer, every 0.5 seconds a LED will toggle on/off
            #print(is_on)

            if is_on:
                GPIO.output(gled, GPIO.LOW) # Turn off green LED
                is_on = False
            else:
                GPIO.output(gled, GPIO.HIGH) # Turn on green LED
                is_on = True
        wsp = isp

    except KeyboardInterrupt: # if you want to end the song early
        break
    # End while
Roomba.Move(0,0) #stop roomba movement

## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly
Roomba.ShutDown() # Shutdown Roomba serial connection
GPIO.cleanup() # Reset GPIO pins for next program
