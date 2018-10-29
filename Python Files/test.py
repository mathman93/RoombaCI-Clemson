import sys, json
sys.path.insert(0, '../')
import RoombaCI_lib
import RPi.GPIO as GPIO
import serial
import sys

GPIO.setmode(GPIO.BCM) # Use BCM pin numbering for GPIO
Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)

Roomba.WakeUp(131)
Roomba.BlinkCleanLight()

if Roomba.Available() > 0:
	x = Roomba.DirectRead(Roomba.Available())

imu = RoombaCI_lib.LSM9DS1_IMU() # Initialize IMU

if sys.argv[1] == "1":
	Roomba.Move(0, 75)
if sys.argv[1] == "2":
	Roomba.Move(75,0)
if sys.argv[1] == "0":
	Roomba.Move(0,0)

# Add the calibrate mag and calibrate accelgyro to function	
LENGTH = 500

magX = [0 for i in range(LENGTH)]
magY = [0 for i in range(LENGTH)]
magZ = [0 for i in range(LENGTH)]

accelX = [0 for i in range(LENGTH)]
accelY = [0 for i in range(LENGTH)]
accelZ = [0 for i in range(LENGTH)]

gyroX = [0 for i in range(LENGTH)]
gyroY = [0 for i in range(LENGTH)]
gyroZ = [0 for i in range(LENGTH)]


for ii in range(LENGTH):
	[a, b, c] = imu.ReadMagRaw()
	magX[ii] = a
	magY[ii] = b
	magZ[ii] = c

	[a, b, c] = imu.ReadAccelRaw()
	accelX[ii] = a
	accelY[ii] = b
	accelZ[ii] = c

	[a, b, c] = imu.ReadGyroRaw()
	gyroX[ii] = a
	gyroY[ii] = b
	gyroZ[ii] = c


print("IMU TESTING", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","w"))

print("magX", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(magX[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\nmagY", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(magY[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\nmagZ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(magZ[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\naccelX", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(accelX[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\naccelY", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(accelY[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\naccelZ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(accelZ[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\ngyroX", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(gyroX[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\ngyroY", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(gyroY[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

print("\ngyroZ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(gyroZ[ii]), file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")
	print(", ", file=open("results_" + sys.argv[1] + "_" + sys.argv[2] + ".csv","a"), end="")

Roomba.Move(0,0)
Roomba.ShutDown()
GPIO.cleanup()
