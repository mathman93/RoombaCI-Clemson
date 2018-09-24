import sys
sys.path.insert(0, '../')
import RoombaCI_lib

Roomba = RoombaCI_lib.Create_2("/dev/ttyS0", 115200)

imu = RoombaCI_lib.LSM9DS1_IMU() # Initialize IMU

LENGTH = 500

readingX = [0 for i in range(LENGTH)]
readingY = [0 for i in range(LENGTH)]
readingZ = [0 for i in range(LENGTH)]

dictionary = {
    "readingXdict" : [],
    "readingYdict" : [],
    "readingZdict" : []
}

def addValue(self, key, value):
    dictionary[key].append(value)

def writeToFile(self):
    with open('results.json','w') as fp:
        json.dump(dictionary,fp)

imu.CalibrateMag()

for ii in range(LENGTH):
    [a, b, c] = imu.ReadMag()
    # a.addValue("readingX")
    # b.addValue("readingY")
    # c.addValue("readingZ")
    readingX[ii] = a
    readingY[ii] = b
    readingZ[ii] = c

addValue("readingXdict",readingX)
addValue("readingYdict",readingY)
addValue("readingZdict",readingZ)

writeToFile()
