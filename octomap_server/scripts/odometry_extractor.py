#!/usr/bin/env python
import pickle
import rospy
import tf
import matplotlib.pyplot as plt
import math
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose 

storedOdom = []
storedOdomCombined = []
f_odom = open("odom_trajectory.txt","w")
f_odomCombined = open("odomCombined_trajectory.txt","w")

def save_to_file(req):
    global storedOdom, storedOdomCombined
    return []

def odomCallback(data):
    global storedOdom
    result = "{}.{} {} {} {} {} {} {} {}\n".format(
        data.header.stamp.secs, data.header.stamp.nsecs,
        data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
        data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
    )
    f_odom.write(result)
    rospy.loginfo("I heard odom")

def odomCombinedCallback(data):
    global storedOdomCombined
    result = "{}.{} {} {} {} {} {} {} {}\n".format(
        data.header.stamp.secs, data.header.stamp.nsecs,
        data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
        data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
    )
    f_odomCombined.write(result)
    rospy.loginfo("I heard odomCombined")

rospy.init_node('odometry_extractor')
rospy.Service('save_odometry_to_file', Empty, save_to_file)
rospy.Subscriber("/base_odometry/odom", Odometry, odomCallback)
rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, odomCombinedCallback)
rospy.spin()