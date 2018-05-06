import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from track_orientation import *

c = 3e8

def load_data(path):
    position_path = 'position_data.npy'
    quaternion_path = 'quaternion_data.npy'
    position_data = np.load(path+position_path)
    quaternion_data = np.load(path+quaternion_path)
    return position_data, quaternion_data

def main():
    '''
    loading data and set configurations
    '''
    position_data, quaternion_data = load_data('data/')
    print(position_data[0], quaternion_data[0])
    N = len(position_data)
    # two satelites at (0,0,20200*1000) and (1e7,1e7,20200*1000)
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
    phase = np.zeros((N,rx_num,2))
    drone_stat = []
    # each drone instance represents a time slot
    for i in range(N):
        drone = Drone(position_data[i],quaternion_data[i])
        drone.rotate(arm_len)
        phase[i,:,0] = drone.phase_calculate(satelite_a)
        phase[i,:,1] = drone.phase_calculate(satelite_b)
        drone_stat.append(drone)

    base_matrix = np.array([drone_stat[0].rx[i]-drone_stat[0].rx[0] for i in range(1,rx_num)])

    # system model, calculating all differentials
    model = Model(N, rx_num)
    model.diff_calculate(phase)
    # form the lse problem
    A = np.inner(base_matrix.dot(drone_stat[1].quaternion.rotation_matrix),satelite_a.los)
    b = satelite_a.wavelength*model.doubleDiffRxTime[1,:,0]-A
    x = np.linalg.pinv(base_matrix).dot(b)
    AA = np.outer(x,satelite_a.los.T)/(satelite_a.los@satelite_a.los.T)
    AAA = np.zeros((3,3))
    AAA[:,0] = normalize(np.cross(AA[:,1],AA[:,2]))
    AAA[:,1] = normalize(np.cross(AA[:,0],AA[:,2]))
    AAA[:,2] = normalize(AA[:,2])
    print(AAA)
    Quaternion(matrix=AAA)
if __name__ == "__main__":
    main()
