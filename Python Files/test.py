import sys, json
sys.path.insert(0, '../')
import RoombaCI_lib

Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)

imu = RoombaCI_lib.LSM9DS1_IMU() # Initialize IMU

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
	
	[a, b, c] = ReadAccelRaw()
	accelX = a
	accelY = b
	accelZ = c
	
	[a, b, c] = ReadGyroRaw()
	gyroX = a
	gyroY = b
	gyroZ = c

print("IMU TESTING", file=open("output.txt","a"))
	
print("magX", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(magX[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("magY", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(magY[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("magZ", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(magZ[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("accelX", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(accelX[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("accelY", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(accelY[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("accelZ", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(accelZ[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("gyroX", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(gyroX[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("gyroY", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(gyroY[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")

print("gyroZ", file=open("output.txt","a"))
for ii in range(LENGTH):
	print("{:.3f}".format(gyroZ[ii]), file=open("output.txt","a"), end="")
	print(", ", file=open("output.txt","a"), end="")
	