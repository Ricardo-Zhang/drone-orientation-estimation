import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from track_orientation import *

def load_data():
    path = 'data/'
    position_path = 'position_data.npy'
    quaternion_path = 'quaternion_data.npy'
    position_data = np.load(path+position_path)
    quaternion_data = np.load(path+quaternion_path)
    return position_data, quaternion_data

def main():
    '''
    loading data and set configurations
    '''
    position_data, quaternion_data = load_data()
    print(position_data[0], quaternion_data[0])
    N = len(position_data)
    satelite_a = satelite(0,0)
    satelite_b = satelite(1e7,1e7)
    rx_num = 4
    arm_len = 0.2 # 0.2 meter from the center to the rx antenna
    '''
    simulating phase based on the data generated
    '''
    phase = np.zeros((N,rx_num,2))
    drone_stat = []
    for i in range(N):
        drone = Drone(position_data[i],quaternion_data[i])
        drone.rotate(arm_len)
        phase[i,:,0] = drone.phase_calculate(satelite_a)
        phase[i,:,1] = drone.phase_calculate(satelite_b)
        drone_stat.append(drone)

    model = Model(N, rx_num)
    model.diff_calculate(phase)


if __name__ == "__main__":
    main()
