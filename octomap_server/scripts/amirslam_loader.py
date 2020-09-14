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

storedPose = PoseArray();
storedConstraint = MarkerArray();

rospy.init_node('amirslam_loader')

posePub = rospy.Publisher('trajectory_pose_array_new', PoseArray, queue_size=10)
constraintPub = rospy.Publisher('constraint_list_new', MarkerArray, queue_size=10)

filename = rospy.get_param("~amirslam_state_filename")

with open(filename) as f:  # Python 3: open(..., 'rb')
    storedPose, storedConstraint = pickle.load(f)

rate = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():
    rospy.loginfo("pub size %d", len(storedPose.poses)) #exactly the same as cartographer trajectory query
    posePub.publish(storedPose)
    constraintPub.publish(storedConstraint)

    rate.sleep()

