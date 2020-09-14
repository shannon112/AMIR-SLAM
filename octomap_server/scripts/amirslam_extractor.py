#!/usr/bin/env python
import pickle
import rospy
import tf
import matplotlib.pyplot as plt
import math
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray

storedPose = None;
storedConstraint = None;

def save_to_file(req):
    global storedPose, storedConstraint
    with open('amirslam_params.pkl', 'w') as f:  # Python 3: open(..., 'wb')
        pickle.dump([storedPose, storedConstraint], f)
    rospy.loginfo("saved to pkl")
    return []

def poseCallback(data):
    global storedPose
    storedPose = data;
    rospy.loginfo("I heard pose array size %d", len(storedPose.poses))

def constraintCallback(data):
    global storedConstraint
    storedConstraint = data;
    rospy.loginfo("I heard constraint array size %d", len(storedConstraint.markers[0].colors))

rospy.init_node('amirslam_extractor')
rospy.Service('save_amir_slam_to_file', Empty, save_to_file)
rospy.Subscriber("trajectory_pose_array_new", PoseArray, poseCallback)
rospy.Subscriber("constraint_list_new", MarkerArray, constraintCallback)
rospy.spin()