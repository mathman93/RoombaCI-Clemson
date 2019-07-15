
''' song for sound
Purpose: Play user song on Roomba
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 6/28/2018
'''
## Import libraries ##
import serial
import time
#import keyboard
import RPi.GPIO as GPIO

import RoombaCI_lib

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO

# LED Pin setup
#GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)

# Wake Up Roomba Sequence
#GPIO.output(gled, GPIO.HIGH) # Turn on green LED to say we are alive
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

def prepNoise():
    # SuperStar Theme (16 notes)
    Roomba.DirectWrite(140) # Header Opcode
    Roomba.DirectWrite(0)   # Song number (0-3)
    Roomba.DirectWrite(100)
    Roomba.DirectWrite(128)

    time.sleep(0.05) # Wait
    
def makeNoise():
    Roomba.DirectWrite(141)
    Roomba.DirectWrite(0)
    
prepNoise()
makeNoise()
time.sleep(1)
Roomba.ShutDown()
GPIO.cleanup()