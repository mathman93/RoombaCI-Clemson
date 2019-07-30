''' swarm4.0.py
Purpose: Code to localize mics using matrix and/or hyperbola methd;
IMPORTANT: Must be run using Python 3 (python3)
Last Modified: 7/24/2019
'''
## Import libraries ##
import serial
import math
import time
import RPi.GPIO as GPIO
import RoombaCI_lib
import multiprocessing
import numpy as np
from multiprocessing import Queue


## Variables and Constants ##
# LED pin numbers
yled = 5
rled = 6
gled = 13
micOne=25#
micTwo=17
micThree=22
micFour=27#FILL IN REAL VALUE
reset=24
statusOne=0
statusTwo=0
statusThree=0
statusFour=0
statusOneTwo=0
statusTwoTwo=0
statusThreeTwo=0
statusFourTwo=0
times=[0,0,0,0]
cSound=343000
x1=0
x2=75
x3=75
x4=-150
y1=0
y2=-129.9038
y3=129.9038
y4=0


wheelToWheel=235###DISTANCE BETWEEN WHEELS
wheelBaseCircumference=wheelToWheel*math.pi###CIRCUMERENCE OF CIRCLE WITH WHEEL TO WHEEL AS DIAMETER
countConversion=72*math.pi/508.8###CONVERTS TICKS TO MM
thetaConvert=72*math.pi/(508.8*235)###CONVERTS TICKS TO RADIANS

## Functions and Definitions ##
''' Displays current date and time to the screen
    '''
def DisplayDateTime():
    # Month day, Year, Hour:Minute:Seconds
    date_time = time.strftime("%B %d, %Y, %H:%M:%S", time.gmtime())
    print("Program run: ", date_time)

    ###RESETS THE FLIP FLOP TO ALLOW FOR NEW SOUND
def timedReset():
    GPIO.output(reset,GPIO.LOW)
    time.sleep(1)
    GPIO.output(reset,GPIO.HIGH)
    
    ###WILL TURN AT SPEED UNTILL THE ROOMBA HAS DISPLACED A TARGET ANGLE
def simpleTurn(ang, speed):#angle in radians, speed in mm/s
    if ang<0:###USED TO DETERMINE DIRECTIONALITY OF SPIN
        speed=-speed
    Roomba.StartQueryStream(43,44)
    leftEncoder,rightEncoder=Roomba.ReadQueryStream(43,44)
    leftInit=leftEncoder
    rightInit=rightEncoder
    target=abs(ang/(2*math.pi)*wheelBaseCircumference)
    Roomba.Move(0, speed)
    while abs(leftEncoder-leftInit)*countConversion<target and abs(rightEncoder-rightInit)*countConversion<target:
        leftEncoder,rightEncoder=Roomba.ReadQueryStream(43,44)
    Roomba.Move(0,0)
    
    ###SAME AS SIMPLE TURN BUT USES MATH TO ACCOUNT FOR ANY DIFFERENCE IN WHEEL ENCODERS.
def complexTurn(ang,speed):
    if ang<0:
        speed=-speed
    Roomba.StartQueryStream(43,44)
    leftEncoder,rightEncoder=Roomba.ReadQueryStream(43,44)
    leftInit=leftEncoder
    rightInit=rightEncoder
    target=abs(ang/(2*math.pi)*wheelBaseCircumference)
    Roomba.Move(0, speed)
    heading=0
    theta=0
    if speed>0:
        while heading<ang:
            leftEncoder,rightEncoder=Roomba.ReadQueryStream(43,44)
            theta=((leftEncoder-leftInit)-(rightEncoder-rightInit))*thetaConvert
            leftInit=leftEncoder
            rightInit=rightEncoder
            heading=heading+theta
    elif speed<0:
        while heading>ang:
            leftEncoder,rightEncoder=Roomba.ReadQueryStream(43,44)
            theta=((leftEncoder-leftInit)-(rightEncoder-rightInit))*thetaConvert
            leftInit=leftEncoder
            rightInit=rightEncoder
            heading=heading+theta
    Roomba.Move(0,0)
    
###DRIVES ROOMBA DISTANCE SPECIFIED BY X AND Y FROM HYPERBOLA METHOD
###ASSUMES TURN TO CORRECT ANGLE HAS ALREADY BEEN MADE.
def driveDist(x,y, speed):
    Roomba.StartQueryStream(43,44)
    targetDist= math.sqrt(x**2+y**2)
    leftEncoder,rightEncoder=Roomba.ReadQueryStream(43,44)
    leftInit=leftEncoder
    rightInit=rightEncoder
    dist=0
    Roomba.Move(speed,0)
    while dist<targetdist:
        leftEncoder,rightEncoder=Roomba.ReadQueryStream(43,44)
        dist=(leftEncoder-leftInit)*countConversion
    Roomba.Move(0,0)

    
def fourMicMatrix():#check variable scope
    m1= np.array([[2*x1-2*x2, 2*y1-2*y2, -2*cSound*(times[1]-times[0])],[2*x1-2*x3, 2*y1-2*y3, -2*cSound*(times[2]-times[0])],[2*x1-2*x4, 2*y1-2*y4, -2*cSound*(times[3]-times[0])]])
    minv=np.linalg.inv(m1)
    m2=np.array([[cSound**2*(times[1]-times[0])**2+x1**2+y1**2-x2**2-y2**2],[cSound**2*(times[2]-times[0])**2+x1**2+y1**2-x3**2-y3**2],[cSound**2*(times[3]-times[0])**2+x1**2+y1**2-x4**2-y4**2]])
    m3=np.matmul(minv,m2)
    return m3
    

## -- Code Starts Here -- ##
# Setup Code #
GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
DisplayDateTime() # Display current date and time

# LED Pin setup
GPIO.setup(yled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(gled, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(micOne, GPIO.IN, pull_up_down= GPIO.PUD_UP)##Check for initial later
GPIO.setup(micTwo, GPIO.IN,  pull_up_down= GPIO.PUD_UP)
GPIO.setup(micThree, GPIO.IN, pull_up_down= GPIO.PUD_UP)
GPIO.setup(micFour, GPIO.IN, pull_up_down= GPIO.PUD_UP)
GPIO.setup(reset, GPIO.OUT, initial=GPIO.LOW)
startloop=0

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
    temp= Roomba.DirectRead(Roomba.Available()) # Clear out Roomba boot-up info
    #print(x) # Include for debugging
print(" ROOMBA Setup Complete")

GPIO.output(reset,GPIO.LOW)
startTime=time.time()
timeBase=time.time()
GPIO.output(reset, GPIO.HIGH)

while True:
    try:
        startloop=time.time()###FOR THE PURPOSE OF ASSESING RUNTIME
        start=time.time()
        while 0 in times:
            statusOne=GPIO.input(micOne)
            statusTwo=GPIO.input(micTwo)
            statusThree=GPIO.input(micThree)
            statusFour=GPIO.input(micFour)
            if statusOne>statusOneTwo:
                times[0]=time.time()
                statusOneTwo=statusOne
            if statusTwo>statusTwoTwo:
                times[1]=time.time()
                statusTwoTwo=statusTwo
            if statusThree>statusThreeTwo:
                times[2]=time.time()
                statusThreeTwo=statusThree
            if statusFour>statusFourTwo:
                times[3]=time.time()
                statusFourTwo=statusFour
        ###PRINT NAH FAM IF NOT ALL THREE MICS ARE HEARD
        if max(times)-min(times)>0.005:
            print("Nah fam")
            print("T1-T2: {0:.7f}".format(1000*(times[0]-times[1])))
            #print("T2-T3: {0:.7f}".format(1000*(times[1]-times[2])))
            print("T1-T3: {0:.7f}".format(1000*(times[0]-times[2])))
            print("T1-T4: {0:.7f}".format(1000*(times[0]-times[3])))
            #times=[0,0,0,0]
        ###PRINTS TIME DIFFERENCES
        else:
            print("T1-T2: {0:.7f}".format(1000*(times[0]-times[1])))
            #print("T2-T3: {0:.7f}".format(1000*(times[1]-times[2])))
            print("T1-T3: {0:.7f}".format(1000*(times[0]-times[2])))
            print("T1-T4: {0:.7f}".format(1000*(times[0]-times[3])))
            ###PRINT OUT ORDER THE MICS WERE HIT
            if times[0]<times[1] and times[0]<times[2]:
                if times[1]<times[2]:
                    print("123")
                else:
                    print("132")
            elif times[1]<times[0] and times[1]<times[2]:
                if times[0]<times[2]:
                    print("213")
                else:
                    print("231")
            elif times[2]<times[1] and times[2]<times[0]:
                if times[1]<times[0]:
                    print("321")
                else:
                    print("312")
            #x,y,distance= fourMicMatrix()
            ans=fourMicMatrix()
            x=ans[0]
            y=ans[1]
            distance=ans[2]
            #print("x: {0:.7f}".format(x))
            #print("y: {0:.7f}".format(y))
            #print("distance: {0:.7f}".format(distance))
            print(x)
            print(y)
            print(distance)
        ###SETTING EVERYTHING BACK UP TO MULTIPROCESS AGAIN
        times=[0,0,0,0]
        statusOneTwo=0
        statusTwoTwo=0
        statusThreeTwo=0
        statusFourTwo=0
        timedReset()
    except KeyboardInterrupt:
        break
GPIO.output(reset,GPIO.LOW)


## -- Ending Code Starts Here -- ##
# Make sure this code runs to end the program cleanly

Roomba.ShutDown() # Shutdown Roomba serial connection
#TERMINATES ANY ACTIVE PROCESSES
# one.terminate()
# two.terminate()
# three.terminate()
GPIO.cleanup() # Reset GPIO pins for next program