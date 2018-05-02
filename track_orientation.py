import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt

class satelite():
    '''
    GPS satellites fly in medium Earth orbit (MEO) at an altitude of approximately 20,200 km
    Operating frequency 1575.42MHz
    '''
    def __init__(self, x, y):
        self.location = np.array((x,y,20200*1000))
        self.frequency = 1575.42e6
        self.wavelength = 3e8/self.frequency
        # self.phase_noise = phase_noise

    def phase_calculate(self, rx_location):
        distance = np.linalg.norm(self.location-rx_location)
        phase = (distance % self.wavelength)*2*np.pi
        # phase += np.random.randn * self.phase_noise
        return phase

class Drone():
    '''
    Quadcopter with 4 GPS receivers
    '''
    def __init__(self, location, quaternion, noise_pwr = 0.01):
        self.location = location
        self.quaternion = Quaternion(quaternion)
        self.rx = np.zeros((4,3))
        self.noise_pwr = 0.01

    def rotate(self, arm_length):
        self.rx = np.array(([arm_length,0,0],[-arm_length,0,0],[0,arm_length,0],
        [0,-arm_length,0]))
        for i in range(4):
            self.rx[i] = self.quaternion.rotate(self.rx[i])
            self.rx[i] += np.random.randn(3)*self.noise_pwr
        self.rx += self.location

    def phase_calculate(self, satelite):
        phase = np.zeros(4)
        for i in range(4):
            phase[i] = satelite.phase_calculate(self.rx[i])
        return phase


class Model():
    '''
    System model:
    measurement:      doubleDiffRxSate
    state transition: doubleDiffRxTime
    '''
    def __init__(self, N, rx_num):
        self.N = N
        self.rx_num = rx_num
        self.singleDiffRx     = np.zeros((N,rx_num,2))
        self.doubleDiffRxSate = np.zeros((N,rx_num))
        self.singleDiffTime   = np.zeros((N,rx_num,2))
        self.doubleDiffRxTime = np.zeros((N,rx_num))

    def diff_calculate(self, phase):
        for j in range(self.rx_num):
            self.singleDiffRx[0][j] = phase[0][j]-phase[0][0]
            self.doubleDiffRxSate[0] = self.singleDiffRx[0,:,0]-self.singleDiffRx[0,:,1]
        for i in range(1,self.N):
            for j in range(self.rx_num):
                self.singleDiffRx[i][j] = phase[i][j]-phase[i][0]
            self.doubleDiffRxSate[i] = self.singleDiffRx[i,:,0]-self.singleDiffRx[i,:,1]
            self.singleDiffTime[i] = phase[i]-phase[i-1]
            self.doubleDiffRxTime[i] = self.doubleDiffRxSate[i]-self.doubleDiffRxSate[i-1]

class Filter():
    def __init__(self, N):
        self.estimation = np.zeros((N,4))
