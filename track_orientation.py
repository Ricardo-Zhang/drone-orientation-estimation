import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt

class Satelite():
    '''
    GPS satellites fly in medium Earth orbit (MEO) at an altitude of approximately 20,200 km
    Operating frequency 1575.42MHz
    '''
    def __init__(self, x, y):
        self.location = np.array((x,y,20200*1000))
        self.frequency = 1575.42e6
        self.c = 3e8
        self.wavelength = self.c/self.frequency
        # self.phase_noise = phase_noise

    def phase_calculate(self, rx_location, phase_noise=0.01):
        distance = np.linalg.norm(self.location-rx_location)
        phase = (distance % self.wavelength)/self.wavelength
        phase += np.random.randn(*phase.shape) * phase_noise
        return phase

    def line_of_sight(self, drone_location):
        self.los = self.location-drone_location
        self.los /= np.sum(self.los)

class Drone():
    '''
    Quadcopter with 4 GPS receivers
    '''
    def __init__(self, location, quaternion, noise_pwr = 0.01):
        self.location = location
        self.quaternion = Quaternion(quaternion)
        self.rx = np.zeros((4,3)) # receiver locations
        self.noise_pwr = noise_pwr

    def rotate(self, arm_length):
        self.rx = np.array(([arm_length,0,0],[-arm_length,0,0],[0,arm_length,0],
        [0,-arm_length,0]))
        for i in range(4):
            self.rx[i] = self.quaternion.rotate(self.rx[i])
            self.rx[i] += self.location
            # self.rx[i] += np.random.randn(3)*self.noise_pwr

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
    R = quaternion.rotation_matrix
    A = (R-R.T)/2
    ro = np.array([A[2,1],A[0,2],A[1,0]]).T
    s = np.linalg.norm(ro)
    c = (R[0,0]+R[1,1]+R[2,2]-1)/2
    theta = np.arctan2(s,c)
    if np.sin(theta) != 0:
        u = ro/s
        r = u*theta
    else:
        r = np.zeros(3)
    return r
