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
    satelite_a = Satelite(0,0)
    satelite_b = Satelite(1e7,1e7)
    satelite_a.line_of_sight(position_data[0])
    satelite_b.line_of_sight(position_data[0])
    wavelength = satelite_a.wavelength
    # the line_of_sight unit vector is approximate as constant vector
    diff_los = satelite_a.los-satelite_b.los
    rx_num = 4
    arm_len = 0.2 # 0.2 meter from the center to the rx antenna
    '''
    simulating phase based on the data generated
    '''
    base_drone = Drone(location=(0,0,0),quaternion=(1,0,0,0))
    base_drone.rotate(arm_length=arm_len)
    base_matrix = np.array([base_drone.rx[i]-base_drone.rx[0] for i in range(1,rx_num)])
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
    for i in range(1,2):
        for j in range(rx_num-1):
            H = np.inner(base_matrix[j],np.cross(drone_stat[i-1].quaternion.rotation_matrix,
            diff_los))
            yy = np.inner(np.inner(base_matrix[j],drone_stat[i-1].quaternion.rotation_matrix),diff_los)
            y = wavelength*model.doubleDiffRxSate[j]
            wavelength*model.doubleDiffRxTime[i]
            print(y-yy)


if __name__ == "__main__":
    main()
