#!/usr/bin/env python
# coding: utf-8

#import ctypes
from ctypes import *
from numpy import arctan
from time import sleep
import sys
import numpy as np
from scipy import linalg


path = "../lib/liblsm9ds1cwrapper.so"
lib = cdll.LoadLibrary(path)

lib.lsm9ds1_create.argtypes = []
lib.lsm9ds1_create.restype = c_void_p

lib.lsm9ds1_begin.argtypes = [c_void_p]
lib.lsm9ds1_begin.restype = None

lib.lsm9ds1_calibrate.argtypes = [c_void_p]
lib.lsm9ds1_calibrate.restype = None

lib.lsm9ds1_gyroAvailable.argtypes = [c_void_p]
lib.lsm9ds1_gyroAvailable.restype = c_int
lib.lsm9ds1_accelAvailable.argtypes = [c_void_p]
lib.lsm9ds1_accelAvailable.restype = c_int
lib.lsm9ds1_magAvailable.argtypes = [c_void_p]
lib.lsm9ds1_magAvailable.restype = c_int

lib.lsm9ds1_readGyro.argtypes = [c_void_p]
lib.lsm9ds1_readGyro.restype = c_int
lib.lsm9ds1_readAccel.argtypes = [c_void_p]
lib.lsm9ds1_readAccel.restype = c_int
lib.lsm9ds1_readMag.argtypes = [c_void_p]
lib.lsm9ds1_readMag.restype = c_int

lib.lsm9ds1_getGyroX.argtypes = [c_void_p]
lib.lsm9ds1_getGyroX.restype = c_float
lib.lsm9ds1_getGyroY.argtypes = [c_void_p]
lib.lsm9ds1_getGyroY.restype = c_float
lib.lsm9ds1_getGyroZ.argtypes = [c_void_p]
lib.lsm9ds1_getGyroZ.restype = c_float

lib.lsm9ds1_getAccelX.argtypes = [c_void_p]
lib.lsm9ds1_getAccelX.restype = c_float
lib.lsm9ds1_getAccelY.argtypes = [c_void_p]
lib.lsm9ds1_getAccelY.restype = c_float
lib.lsm9ds1_getAccelZ.argtypes = [c_void_p]
lib.lsm9ds1_getAccelZ.restype = c_float

lib.lsm9ds1_getMagX.argtypes = [c_void_p]
lib.lsm9ds1_getMagX.restype = c_float
lib.lsm9ds1_getMagY.argtypes = [c_void_p]
lib.lsm9ds1_getMagY.restype = c_float
lib.lsm9ds1_getMagZ.argtypes = [c_void_p]
lib.lsm9ds1_getMagZ.restype = c_float

lib.lsm9ds1_calcGyro.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcGyro.restype = c_float
lib.lsm9ds1_calcAccel.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcAccel.restype = c_float
lib.lsm9ds1_calcMag.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcMag.restype = c_float

if __name__ == "__main__":
    imu = lib.lsm9ds1_create()
    mag = Magnetometer()
    lib.lsm9ds1_begin(imu)
    if lib.lsm9ds1_begin(imu) == 0:
        print("Failed to communicate with LSM9DS1.")
        quit()
    lib.lsm9ds1_calibrate(imu)










class Magnetometer(object):
    ''' Magnetometer class with calibration capabilities.

        Parameters
        ----------
        sensor : str
            Sensor to use.
        bus : int
            Bus where the sensor is attached.
        F : float (optional)
            Expected earth magnetic field intensity, default=1.
    '''

    def __init__(self, F=1.):

        # initialize values
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)

    def read(self,imu):
        ''' Get a sample.

            Returns
            -------
            s : list
                The sample in uT, [x,y,z] (corrected if performed calibration).
        '''

	

        while lib.lsm9ds1_magAvailable(imu) == 0:
            pass
        lib.lsm9ds1_readMag(imu)

        mx = lib.lsm9ds1_getMagX(imu)
        my = lib.lsm9ds1_getMagY(imu)
        mz = lib.lsm9ds1_getMagZ(imu)

        cmx = lib.lsm9ds1_calcMag(imu, mx)
        cmy = lib.lsm9ds1_calcMag(imu, my)
        cmz = lib.lsm9ds1_calcMag(imu, mz)

        s = np.array([x,y,z]).reshape(3, 1)
        s = np.dot(self.A_1, s - self.b)
        return [s[0,0], s[1,0], s[2,0]]

    def calibrate(self,imu):
        ''' Performs calibration. '''

        print('Collecting samples (Ctrl-C to stop and perform calibration)')

        try:
            s = []
            n = 0
            while True:
                s.append(self.read(self,imu))
                n += 1
                sys.stdout.write('\rTotal: %d' % n)
                sys.stdout.flush()
        except KeyboardInterrupt:
            pass

        # ellipsoid fit
        s = np.array(s).T
        M, n, d = self.__ellipsoid_fit(s)

        # calibration parameters
        # note: some implementations of sqrtm return complex type, taking real
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) *
                           linalg.sqrtm(M))

    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadric-form parameters
        M = np.array([[v_1[0], v_1[3], v_1[4]],
                      [v_1[3], v_1[1], v_1[5]],
                      [v_1[4], v_1[5], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d


























