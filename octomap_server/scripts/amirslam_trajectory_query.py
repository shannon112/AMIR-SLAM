#!/usr/bin/env python
import pickle
import matplotlib.pyplot as plt
import math
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import sys

state_filename = sys.argv[1]

storedPose = PoseArray();
storedConstraint = MarkerArray();

with open(state_filename) as f:  # Python 3: open(..., 'rb')
    storedPose, storedConstraint = pickle.load(f)

fw = open('amir_trajectory.txt','w')
for pose in storedPose.poses:
    result = "{} {} {} {} {} {} {}\n".format(pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    fw.write(result)