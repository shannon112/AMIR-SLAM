#!/usr/bin/env python
import matplotlib.pyplot as plt

x_list_5 = []
y_list_5 = []
f = open("/home/shannon/Documents/master_thesis/2012-04-06-11-15-29_part1_floor2.gt.laser.poses", "r") 
for line in f:
    elements = line.split(",")
    x_list_5.append(elements[1])
    y_list_5.append(elements[2])
plt.plot(x_list_5, y_list_5,label="Ground truth")

'''
x_list_3 = []
y_list_3 = []
f = open("/home/shannon/Documents/master_thesis/amir_v3/odom_trajectory.txt.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_3.append(elements[1])
    y_list_3.append(elements[2])
plt.plot(x_list_3, y_list_3,label="Raw odometry")

x_list_4 = []
y_list_4 = []
f = open("/home/shannon/Documents/master_thesis/amir_v3/odomCombined_trajectory.txt.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_4.append(elements[1])
    y_list_4.append(elements[2])
plt.plot(x_list_4, y_list_4,label="Filtered odometry")

'''

x_list_1 = []
y_list_1 = []
f = open("/home/shannon/Documents/master_thesis/rgbdv2_odom/trajectory_estimate.txt.flatten.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_1.append(elements[1])
    y_list_1.append(elements[2])
plt.plot(x_list_1, y_list_1,label="RGBDSLAMv2")

x_list_2 = []
y_list_2 = []
f = open("/home/shannon/Documents/master_thesis/rgbdv2_rgbd/trajectory_estimate.txt.flatten.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_2.append(elements[1])
    y_list_2.append(elements[2])
plt.plot(x_list_2, y_list_2,label="RGBDSLAMv2 (RGBD only)")

x_list_6 = []
y_list_6 = []
f = open("/home/shannon/Documents/master_thesis/rtab_odom/poses_rgbd.txt.flatten.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_6.append(elements[1])
    y_list_6.append(elements[2])
plt.plot(x_list_6, y_list_6,label="RTAB-Map")


x_list_7 = []
y_list_7 = []
f = open("/home/shannon/Documents/master_thesis/rtab_rgbd/poses_rgbd.txt.flatten.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_7.append(elements[1])
    y_list_7.append(elements[2])
plt.plot(x_list_7, y_list_7,label="RTAB-Map (RGBD only)")

x_list_8 = []
y_list_8 = []
f = open("/home/shannon/Documents/master_thesis/amir_v3/amir_trajectory.txt.padded.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_8.append(elements[1])
    y_list_8.append(elements[2])
plt.plot(x_list_8, y_list_8,label="Our approach")
#c='lime'

#plt.suptitle('Trajectories from A to B in case4')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()
plt.grid(True)

plt.show()
