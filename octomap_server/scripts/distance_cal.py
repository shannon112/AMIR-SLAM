#!/usr/bin/env python
import pickle
import tf
import matplotlib.pyplot as plt
import math 
import sys

filename = sys.argv[1]

f = open (filename, "r")

prev_x = 0
prev_y = 0
distance = 0
for i, line in enumerate(f):
    items = line.strip().split(" ")
    x = float(items[1])
    y = float(items[2])
    segment = math.sqrt((x-prev_x)**2 + (y-prev_y)**2 )
    distance += segment
    prev_x = x
    prev_y = y

print ("distance",distance)

