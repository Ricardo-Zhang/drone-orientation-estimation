import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
'''
Simulation of a drone with 4 gps receivers
- The gps receivers have a distance of 10 centimeters to the drone center
- The location is simulated equally distributed in a 5x5x5 cube
- Observations from the receivers are Gaussian noisy estimation of (x,y,z)
'''

def simulate(N, num_rx, dist, noise):
    original_loc = np.array(([dist,dist,0],[dist,-dist,0],[-dist,dist,0],[-dist,-dist,0]))
    gps_loc = np.zeros((N,num_rx,3))
    gps_loc_noisy = np.zeros((N,num_rx,3))
    orientation = []
    orientation.append(Quaternion())

    # initial state
    gps_loc[0] = original_loc
    for j in range(num_rx):
        gps_loc_noisy[0,j,:] = gps_loc[0,j,:]+np.random.normal(0,noise,3)

    for i in range(1,N):
        for j in range(num_rx):
            rot = Quaternion.random()
            orientation.append(orientation[-1]*rot)
            gps_loc[i,j,:] = rot.rotate(gps_loc[i-1,j,:])
            gps_loc_noisy[i,j,:] = gps_loc[i,j,:]+np.random.normal(0,noise,3)
    return orientation, gps_loc, gps_loc_noisy


def visualize(idx, orientation, gps_loc, gps_loc_noisy, num_rx):
    '''
    visualize the locations of the drone itself and the gps receivers
    - calculate the directions of roll pitch yaw and plot
    - add the drone center location
    - add the gps receivers location (with / without noise)
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.scatter(0,0,0,marker='*')
    colors = ['r','g','b','y']
    axis_loc = np.array(([1,0,0],[0,1,0],[0,0,1]))*0.1
    for i in range(3):
        end_pt = orientation[idx].rotate(axis_loc[i])
        directions = np.array(([0,end_pt[0]],[0,end_pt[1]],[0,end_pt[2]],))
        ax.plot(directions[0],directions[1],directions[2])
    for j in range(num_rx):
        ax.scatter(gps_loc[idx,j,0],gps_loc[idx,j,1],gps_loc[idx,j,2],c=colors[j],marker='o')
        ax.scatter(gps_loc_noisy[idx,j,0],gps_loc_noisy[idx,j,1],gps_loc_noisy[idx,j,2],c=colors[j], marker='.')
    plt.show()


def main():
    N = int(1e3)  # iterations
    num_rx = 4  # num of receivers
    dist = 0.1  # the distance
    noise = 0.01  # Additive Gaussian Noise on gps rx
    orientation, gps_loc, gps_loc_noisy = simulate(N, num_rx, dist, noise)

    visualize(1,orientation, gps_loc, gps_loc_noisy, num_rx)

    path = 'data/'
    gps_loc_file = 'gps_Locations.npy'
    gps_loc_noisy_file = 'Observations.npy'
    # drone_loc_file = 'Drone_Locations.npy'
    drone_orie_delta_file = 'drone_orie_deltantations.npy'
    np.save(path+gps_loc_file, gps_loc)
    np.save(path+gps_loc_noisy_file, gps_loc_noisy)
    np.save(path+drone_orie_delta_file, drone_orie_delta)

    for i in range(N):
        print(repr(orientation[i]))

if __name__ == "__main__":
    main()
