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

print("magX")
for ii in range(LENGTH)
	print("{:.3f}".format(magX[ii]))
	print(", ")

print("magY")
for ii in range(LENGTH)
	print("{:.3f}".format(magY[ii]))
	print(", ")

print("magZ")
for ii in range(LENGTH)
	print("{:.3f}".format(magZ[ii]))
	print(", ")

print("accelX")
for ii in range(LENGTH)
	print("{:.3f}".format(accelX[ii]))
	print(", ")

print("accelY")
for ii in range(LENGTH)
	print("{:.3f}".format(accelY[ii]))
	print(", ")

print("accelZ")
for ii in range(LENGTH)
	print("{:.3f}".format(accelZ[ii]))
	print(", ")

print("gyroX")
for ii in range(LENGTH)
	print("{:.3f}".format(gyroX[ii]))
	print(", ")

print("gyroY")
for ii in range(LENGTH)
	print("{:.3f}".format(gyroY[ii]))
	print(", ")

print("gyroZ")
for ii in range(LENGTH)
	print("{:.3f}".format(gyroZ[ii]))
	print(", ")