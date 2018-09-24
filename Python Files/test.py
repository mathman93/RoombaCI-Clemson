import sys
sys.path.insert(0, '../')
import RoombaCI_lib

Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)

imu = RoombaCI_lib.LSM9DS1_IMU # Initialize IMU

LENGTH = 500

readingX = [0 for i in range(LENGTH)]
readingY = [0 for i in range(LENGTH)]
readingZ = [0 for i in range(LENGTH)]

for ii in range(LENGTH):
    [readingX[ii], readingY[ii], readingZ[ii]] = imu.ReadMagRaw(imu)

for ii in range(LENGTH):
    print(reading[ii])
