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


def rotate(orientation, original_location):
    '''
    Rotate the 3d location by the orientation
    '''
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]
    rotation_mat = np.array((
    [np.cos(roll)*np.cos(pitch),np.cos(roll)*np.sin(pitch)*np.sin(yaw)-np.sin(roll)*np.cos(yaw),
    np.cos(roll)*np.sin(pitch)*np.cos(yaw)+np.sin(roll)*np.sin(yaw)],
    [np.sin(roll)*np.cos(pitch),np.sin(roll)*np.sin(pitch)*np.sin(yaw)+np.cos(roll)*np.cos(yaw),
    np.sin(roll)*np.sin(pitch)*np.cos(yaw)-np.cos(roll)*np.sin(yaw)],
    [-np.sin(pitch),np.cos(pitch)*np.sin(roll),np.cos(pitch)*np.cos(yaw)]))
    rotated_loc = np.dot(rotation_mat,original_location)
    return rotated_loc


def visualize(idx):
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
        directions = np.array((
        [0,rotate(drone_orie_delta[idx],axis_loc[i])[0]],
        [0,rotate(drone_orie_delta[idx],axis_loc[i])[1]],
        [0,rotate(drone_orie_delta[idx],axis_loc[i])[2]],
        ))
        ax.plot(directions[0],directions[1],directions[2])
    for j in range(num_rx):
        ax.scatter(gps_loc[idx,j,0],gps_loc[idx,j,1],gps_loc[idx,j,2],c=colors[j],marker='o')
        ax.scatter(gps_loc_noisy[idx,j,0],gps_loc_noisy[idx,j,1],gps_loc_noisy[idx,j,2],c=colors[j], marker='.')
    plt.show()


N = int(1e3)
num_rx = 4
dist = 0.1  # the distance
noise = 0.1  # Additive Gaussian Noise on gps rx
original_loc = np.array(([dist,dist,0],[dist,-dist,0],[-dist,dist,0],[-dist,-dist,0]))
drone_orie_delta = 0.1*np.random.uniform(-np.pi,np.pi,(N,3))
drone_orie = np.zeros((N,3))
gps_loc = np.zeros((N,num_rx,3))
gps_loc_noisy = np.zeros((N,num_rx,3))

gps_loc[0] = original_loc
for j in range(num_rx):
    gps_loc_noisy[0,j,:] = gps_loc[0,j,:]+np.random.normal(0,noise,3)


for i in range(1,N):
    for j in range(num_rx):
        gps_loc[i,j,:] = rotate(drone_orie_delta[i],gps_loc[i-1,j,:])
        gps_loc_noisy[i,j,:] = gps_loc[i,j,:]+np.random.normal(0,noise,3)

visualize(1)
path = 'data/'
gps_loc_file = 'gps_Locations.npy'
gps_loc_noisy_file = 'Observations.npy'
# drone_loc_file = 'Drone_Locations.npy'
drone_orie_delta_file = 'drone_orie_deltantations.npy'
np.save(path+gps_loc_file, gps_loc)
np.save(path+gps_loc_noisy_file, gps_loc_noisy)
# np.save(path+drone_loc_file, drone_loc)
np.save(path+drone_orie_delta_file, drone_orie_delta)
