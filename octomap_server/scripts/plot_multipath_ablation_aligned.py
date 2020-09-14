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

x_list_3 = []
y_list_3 = []
f = open("/home/shannon/Documents/master_thesis/amir_v3/cartographer_trajectory.txt.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_3.append(elements[1])
    y_list_3.append(elements[2])
plt.plot(x_list_3, y_list_3,label="Cartographer2D")

x_list_4 = []
y_list_4 = []
f = open("/home/shannon/Documents/master_thesis/amir_v3/amir_trajectory.txt.padded.aligned", "r") 
for line in f:
    elements = line.split(" ")
    x_list_4.append(elements[1])
    y_list_4.append(elements[2])
plt.plot(x_list_4, y_list_4,label="Our approach")

#plt.suptitle('Trajectories from A to B in case4')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()
plt.grid(True)

plt.show()
