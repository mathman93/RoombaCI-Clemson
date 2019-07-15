
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

gled=13
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

def prepNoise():
    # SuperStar Theme (4 notes)
    Roomba.DirectWrite(140) # Header Opcode
    Roomba.DirectWrite(0)   # Song number (0-3)
    Roomba.DirectWrite(40)
    Roomba.DirectWrite(128)

    time.sleep(0.05) # Wait
    
def makeNoise():
    GPIO.output(gled, GPIO.LOW)
    Roomba.DirectWrite(141)
    Roomba.DirectWrite(0)
    GPIO.output(gled, GPIO.HIGH)
    
#prepNoise()
#makeNoise()
    # SuperStar Theme (4 notes)
while True:
    #try:
        note1=int(input("First Note:"))
        note2=int(input("Second Note:"))
        note3=int(input("Third Note:"))
        note4=int(input("Fourth Note:"))
        note5=int(input("Fifth Note:"))
        note6=int(input("Sixth Note:"))
        note7=int(input("Seventh Note:"))
        note8=int(input("Eight Note:"))
        Roomba.DirectWrite(140) # Header Opcode
        Roomba.DirectWrite(0)   # Song number (0-3)
        Roomba.DirectWrite(8)
        Roomba.DirectWrite(note1)
        Roomba.DirectWrite(6)
        Roomba.DirectWrite(note2)
        Roomba.DirectWrite(6)
        Roomba.DirectWrite(note3)
        Roomba.DirectWrite(6)
        Roomba.DirectWrite(note4)
        Roomba.DirectWrite(6)
        Roomba.DirectWrite(note5)
        Roomba.DirectWrite(6)
        Roomba.DirectWrite(note6)
        Roomba.DirectWrite(6)
        Roomba.DirectWrite(note7)
        Roomba.DirectWrite(6)
        Roomba.DirectWrite(note8)
        Roomba.DirectWrite(6)

        time.sleep(0.05) # Wait
        GPIO.output(gled, GPIO.LOW)
        time.sleep(1) # Wait
        Roomba.DirectWrite(141)
        Roomba.DirectWrite(0)
        GPIO.output(gled, GPIO.HIGH)
        time.sleep(1)
    #except KeyboardInterrupt:
     #   break
Roomba.ShutDown()
GPIO.cleanup()