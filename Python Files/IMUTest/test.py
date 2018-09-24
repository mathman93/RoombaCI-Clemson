import sys
sys.path.insert(0, '../')
import RoombaCI_lib

Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)

imu = RoombaCI_lib.LSM9DS1_IMU() # Initialize IMU

for ii in range(500):
    reading[ii] = imu.ReadMagRaw

for ii in range(500):
    print(reading[ii])
