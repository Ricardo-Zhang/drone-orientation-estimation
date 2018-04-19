import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
'''
Simulation of a drone with 4 GPS receivers
- The GPS receivers have a distance of 10 centimeters to the drone center
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
    - add the GPS receivers location (with / without noise)
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.scatter(drone_loc[idx][0],drone_loc[idx][1],drone_loc[idx][2],marker='*')
    colors = ['r','g','b','y']
    axis_loc = np.array(([1,0,0],[0,1,0],[0,0,1]))*0.1
    for i in range(3):
        directions = np.array((
        [drone_loc[idx][0],drone_loc[idx][0]+rotate(drone_orie[idx],axis_loc[i])[0]],
        [drone_loc[idx][1],drone_loc[idx][1]+rotate(drone_orie[idx],axis_loc[i])[1]],
        [drone_loc[idx][2],drone_loc[idx][2]+rotate(drone_orie[idx],axis_loc[i])[2]],
        ))
        ax.plot(directions[0],directions[1],directions[2])
    for j in range(num_rx):
        ax.scatter(GPS_loc[idx,j,0],GPS_loc[idx,j,1],GPS_loc[idx,j,2],c=colors[j])
        ax.scatter(GPS_loc_noisy[idx,j,0],GPS_loc_noisy[idx,j,1],GPS_loc_noisy[idx,j,2],c=colors[j])
    plt.show()


N = int(1e3)
num_rx = 4
dist = 0.1  # the distance
noise = 0.2  # Additive Gaussian Noise on GPS rx
original_loc = np.array(([dist,dist,0],[dist,-dist,0],[-dist,dist,0],[-dist,-dist,0]))
drone_loc = np.random.uniform(0,5,(N,3))
drone_orie = np.random.uniform(-np.pi,np.pi,(N,3))
GPS_loc = np.zeros((N,num_rx,3))
GPS_loc_noisy = np.zeros((N,num_rx,3))

for i in range(N):
    for j in range(num_rx):
        GPS_loc[i,j,:] = drone_loc[i,:]+rotate(drone_orie[i],original_loc[j])
        GPS_loc_noisy[i,j,:] = GPS_loc[i,j,:]+np.random.normal(0,noise,3)
visualize(0)
