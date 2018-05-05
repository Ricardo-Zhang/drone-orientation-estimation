#!/usr/bin/env python
import rospy
import numpy as np
import io, json
import pickle
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped, TransformStamped, PoseWithCovarianceStamped, Vector3Stamped

position = []
quaternion = []
timeslot = []
count = 0

def callback(pose):
    '''
    compare and notify if the location is near the target location
    '''
    global count
    posi_array = np.array((pose.pose.position.x,pose.pose.position.y,pose.pose.position.z))
    quat_array = np.array((pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))
    position.append(posi_array)
    quaternion.append(quat_array)
    timeslot.append(pose.header.stamp.to_sec())
    count += 1
    print(count)
        
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback)
    rospy.spin()

try:
    listener()
except KeyboardInterrupt:
    print('Exiting...')
finally:
    position_array = np.array(position)
    quaternion_array = np.array(quaternion)
    with open("position_data.npy","w") as p_f:
        np.save(p_f, position_array)
    with open("quaternion_data.npy", "w") as q_f:
        np.save(q_f, quaternion_array)
    with open("time_data.txt", "w") as t_f:
        pickle.dump(timeslot,t_f)
