import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from track_orientation import *

c = 3e8


def load_data(path):
    position_path = 'position_data.npy'
    quaternion_path = 'quaternion_data.npy'
    position_data = np.load(path + position_path)
    quaternion_data = np.load(path + quaternion_path)
    return position_data,quaternion_data


def transition_theta_cal(base_mat,drone,satelite_a,satelite_b,doubleDiffRxT):
    '''
    solve transitional vector via LSE from doubleDiffRxTime
    drone: instance of drone: drone_stat[i]
    satelite: satelite_a or b
    doubleDiffRxT: model.doubleDiffRxTime[i,:,0] for sat a, [i,:,1] for b
    '''
    t_los = drone.quaternion.rotation_matrix @ satelite_a.los
    A_mat = cross_prod_mat(t_los)
    A_mat = base_mat.T @ A_mat
    t_los = drone.quaternion.rotation_matrix @ satelite_a.los
    B_mat = cross_prod_mat(t_los)
    A = np.concatenate((A_mat,B_mat),axis=0)
    b = satelite_a.wavelength * np.concatenate((doubleDiffRxT[:,0],doubleDiffRxT[:,1]),axis=0)
    x = np.linalg.pinv(A) @ b
    return x


def theta_cal(base_mat,drone,satelite_a,satelite_b,doubleDiffRxSat):
    t_los = drone.quaternion.rotation_matrix @ (satelite_a.los - satelite_b.los)
    A_mat = cross_prod_mat(t_los)
    A_mat = base_mat.T @ A_mat
    b = satelite_a.wavelength*doubleDiffRxSat - base_mat @ drone.quaternion.rotation_matrix @ (satelite_a.los - satelite_b.los)
    x = np.linalg.pinv(A_mat)@b
    return x

def cross_prod_mat(a):
    mat = np.array([[0,-a[2],a[1]],[a[2],0,a[0]],[-a[1],a[0],0]])
    return mat


def main():
    '''
    loading data and set configurations
    '''
    print('running')
    position_data,quaternion_data = load_data('data/')
    print(position_data[0],quaternion_data[0])
    N = len(position_data)
    # two satelites at (0,0,20200*1000) and (1e7,1e7,20200*1000)
    phase_noise = 1e-3
    satelite_a = Satelite(-1e7,-1e7,phase_noise=phase_noise)
    satelite_b = Satelite(1e7,1e7,phase_noise=phase_noise)
    satelite_a.line_of_sight(position_data[0])
    satelite_b.line_of_sight(position_data[0])
    wavelength = satelite_a.wavelength
    noise = wavelength**2*phase_noise
    # the line_of_sight unit vector is approximate as constant vector
    diff_los = satelite_a.los - satelite_b.los
    rx_num = 4
    arm_len = 0.2  # 0.2 meter from the center to the rx antenna
    '''
    simulating phase based on the data generated
    '''
    phase = np.zeros((N,rx_num,2))
    drone_stat = []
    # each drone instance represents a time slot
    for i in range(N):
        drone = Drone(position_data[i],quaternion_data[i],rx_num=rx_num)
        drone.rotate(arm_len)
        phase[i,:,0] = drone.phase_calculate(satelite_a)
        phase[i,:,1] = drone.phase_calculate(satelite_b)
        drone_stat.append(drone)

    base_mat = np.array([drone_stat[0].rx[i] - drone_stat[0].rx[0] for i in range(1,rx_num)])
    # system model, calculating all differentials
    model = Model(N,rx_num)
    model.diff_calculate(phase)

    quaternion_est = [drone_stat[0].quaternion]

    for i in range(1,10):
        delta_theta = transition_theta_cal(base_mat,drone_stat[i - 1],satelite_a,satelite_b,
                                           model.doubleDiffRxTime[i,:,:])
        delta_quat = vec2quat(delta_theta)
        quaternion_est.append(delta_quat*quaternion_est[-1]+np.random.randn(4)*noise)
        delta_theta_forward = quat2vec(quaternion_est[-1])

        theta = theta_cal(base_mat,drone_stat[i],satelite_a,satelite_b ,model.doubleDiffRxSate[i,:])


if __name__ == "__main__":
    main()
