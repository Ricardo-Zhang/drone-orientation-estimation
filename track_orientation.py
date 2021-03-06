import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt

class Satelite():
    '''
    GPS satellites fly in medium Earth orbit (MEO) at an altitude of approximately 20,200 km
    Operating frequency 1575.42MHz
    '''
    def __init__(self, x, y, phase_noise=1e-3):
        self.location = np.array((x,y,20200*1000))
        self.frequency = 1575.42e6
        self.c = 3e8
        self.wavelength = self.c/self.frequency
        self.phase_noise = phase_noise
        # self.phase_noise = phase_noise

    def phase_calculate(self, rx_location):
        distance = np.linalg.norm(self.location-rx_location)
        phase = (distance % self.wavelength)/self.wavelength
        phase += np.random.randn(*phase.shape) * self.phase_noise
        return phase

    def line_of_sight(self, drone_location):
        self.los = self.location-drone_location
        self.los /= np.linalg.norm(self.los)

class Drone():
    '''
    Quadcopter with 4 GPS receivers
    '''
    def __init__(self, location, quaternion, rx_num=2, noise_pwr = 0.01):
        self.location = location
        self.quaternion = Quaternion(quaternion)
        self.rx_num = rx_num
        self.rx = np.zeros((rx_num,3)) # receiver locations
        self.noise_pwr = noise_pwr

    def rotate(self, arm_length):
        self.rx = np.array(([arm_length,0,0],[-arm_length,0,0],[0,arm_length,0],[0,-arm_length,0]))
        for i in range(self.rx_num):
            self.rx[i] = self.quaternion.rotate(self.rx[i])
            self.rx[i] += self.location
            # self.rx[i] += np.random.randn(3)*self.noise_pwr

    def phase_calculate(self, satelite):
        phase = np.zeros(self.rx_num)
        for i in range(self.rx_num):
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
        self.singleDiffRx     = np.zeros((N,rx_num,2)) # will be clipped to N*(rx_num-1)*2
        self.doubleDiffRxSate = np.zeros((N,rx_num)) # will be clipped to N*(rx_num-1)
        self.singleDiffTime   = np.zeros((N,rx_num,2))
        self.doubleDiffRxTime = np.zeros((N,rx_num,2)) # will be clipped to N*(rx_num-1)

    def diff_calculate(self, phase):
        for j in range(self.rx_num):
            self.singleDiffRx[0][j] = phase[0][j]-phase[0][0]
            self.doubleDiffRxSate[0] = self.singleDiffRx[0,:,0]-self.singleDiffRx[0,:,1]
        for i in range(1,self.N):
            for j in range(self.rx_num):
                self.singleDiffRx[i][j] = phase[i][j]-phase[i][0]
            self.doubleDiffRxSate[i] = self.singleDiffRx[i,:,0]-self.singleDiffRx[i,:,1]
            self.singleDiffTime[i] = phase[i]-phase[i-1]
            self.doubleDiffRxTime[i] = self.singleDiffRx[i]-self.singleDiffRx[i-1]
        self.singleDiffRx = self.singleDiffRx[:,1:,:]
        self.doubleDiffRxSate = self.doubleDiffRxSate[:,1:]
        self.doubleDiffRxTime = self.doubleDiffRxTime[:,1:,:]

def normalize(vector):
    return vector/np.linalg.norm(vector)

def quat2vec(quaternion):
    ax = quaternion.axis
    theta = quaternion.angle
    v = normalize(ax)*theta
    return v

def vec2quat(rotate_vector):
    theta = np.linalg.norm(rotate_vector)
    ax = normalize(rotate_vector)
    q = Quaternion(axis=ax, radians=theta)
    return q
