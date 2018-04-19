from ctypes import *
import time
from math import atan2

class Magnetometer:

    def __init__(self):
        # initalize magnetometer
        path = "/home/pi/RoombaCI/LSM9DS1_RaspberryPi_Library-master/lib/liblsm9ds1cwrapper.so"
        self.lib = cdll.LoadLibrary(path)

        self.lib.lsm9ds1_create.argtypes = []
        self.lib.lsm9ds1_create.restype = c_void_p

        self.lib.lsm9ds1_begin.argtypes = [c_void_p]
        self.lib.lsm9ds1_begin.restype = None

        self.lib.lsm9ds1_calibrate.argtypes = [c_void_p]
        self.lib.lsm9ds1_calibrate.restype = None

        self.lib.lsm9ds1_magAvailable.argtypes = [c_void_p]
        self.lib.lsm9ds1_magAvailable.restype = c_int

        self.lib.lsm9ds1_readMag.argtypes = [c_void_p]
        self.lib.lsm9ds1_readMag.restype = c_int

        self.lib.lsm9ds1_getMagX.argtypes = [c_void_p]
        self.lib.lsm9ds1_getMagX.restype = c_float
        self.lib.lsm9ds1_getMagY.argtypes = [c_void_p]
        self.lib.lsm9ds1_getMagY.restype = c_float

        self.lib.lsm9ds1_calcMag.argtypes = [c_void_p, c_float]
        self.lib.lsm9ds1_calcMag.restype = c_float

        self.imu = self.lib.lsm9ds1_create()
        self.lib.lsm9ds1_begin(self.imu)

        if self.lib.lsm9ds1_begin(self.imu) == 0:
            print("Failed to communicate with LSM9DS1.")
            quit()
        self.lib.lsm9ds1_calibrate(self.imu)
        self.x_min = 10
        self.x_max = -10
        self.y_min = 10
        self.y_max = -10

    def Calibrate(self):
        # Rooma should be spinning when this is called
        for i in range(0,1500):
            while self.lib.lsm9ds1_magAvailable(self.imu) == 0:
                pass
            self.lib.lsm9ds1_readMag(self.imu)

            mx = self.lib.lsm9ds1_getMagX(self.imu)
            my = self.lib.lsm9ds1_getMagY(self.imu)

            cmx = self.lib.lsm9ds1_calcMag(self.imu, mx)
            cmy = self.lib.lsm9ds1_calcMag(self.imu, my)

            if cmx < self.x_min:
                self.x_min = cmx
            if cmx > self.x_max:
                self.x_max = cmx
            if cmy < self.y_min:
                self.y_min = cmy
            if cmy > self.y_max:
                self.y_max = cmy

    def read(self):
        while self.lib.lsm9ds1_magAvailable(self.imu) == 0:
            pass
        self.lib.lsm9ds1_readMag(self.imu)

        mx = self.lib.lsm9ds1_getMagX(self.imu)
        my = self.lib.lsm9ds1_getMagY(self.imu)

        cmx = self.lib.lsm9ds1_calcMag(self.imu, mx)
        cmy = self.lib.lsm9ds1_calcMag(self.imu, my)

        return [cmx-(self.x_max+self.x_min)/2, cmy-(self.y_max+self.y_min)/2]
