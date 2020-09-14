#!/usr/bin/env python
import pickle
import matplotlib.pyplot as plt
import math
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import sys

trajectory_filename_amir = sys.argv[1]
trajectory_filename_cart = sys.argv[2]

f_amir = open(trajectory_filename_amir, "r")
f_cart = open(trajectory_filename_cart, "r")

fw = open("amir_trajectory_padded.txt", "w")

time = []

for line in f_cart:
    time.append(line.split(" ")[0])
print(len(time))

for i, line in enumerate(f_amir):
    result = time[i]+" "+line
    fw.write(result)
print(i+1)
