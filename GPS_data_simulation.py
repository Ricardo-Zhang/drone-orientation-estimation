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

def load_data():
    path = 'data/'
    position_path = 'position_data.npy'
    quaternion_path = 'quaternion_data.npy'
    position_data = np.load(path+position_path)
    quaternion_data = np.load(path+quaternion_path)
    return position_data, quaternion_data

def main():
    position_data, quaternion_data = load_data()
    print(position_data[0], quaternion_data[0])
    N = len(position_data)
    satelite_a = satelite(0,0)
    satelite_b = satelite(1e7,1e7)
    rx_num = 4
    arm_len = 0.2 # 0.2 meter from the center to the rx antenna

    phase = np.zeros((N,rx_num,2))
    drone_stat = []
    for i in range(N):
        drone = Drone(position_data[i],quaternion_data[i])
        drone.rotate(arm_len)
        phase[i,:,0] = drone.phase_calculate(satelite_a)
        phase[i,:,1] = drone.phase_calculate(satelite_b)
        drone_stat.append(drone)
        
    print(phase[0])

if __name__ == "__main__":
    main()
