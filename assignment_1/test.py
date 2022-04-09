from numpy import multiply, any
import math
import rospy
import actionlib
from math import atan2, pi, cos, sin, sqrt

f = open("catkin_ws/src/sc627_assignments/assignment_1/input.txt", "r")
l = f.readline()
start = [float(l[0]), float(l[2])]
l = f.readline()
goal = [float(l[0]), float(l[2])]
l = f.readline()
step = float(l)
l = f.readline()
l = f.readline()
p1 = [float(l[0]), float(l[2])]
l = f.readline()
p2 = [float(l[0]), float(l[2])]
l = f.readline()
p3 = [float(l[0]), float(l[2])]
l = f.readline()
l = f.readline()
p4 = [float(l[0]), float(l[2])]
l = f.readline()
p5 = [float(l[0]), float(l[2])]
l = f.readline()
p6 = [float(l[0]), float(l[2])]
f.close()

P1 = [p1, p2, p3]
P2 = [p4, p5, p6]

a=P1
b=[1,0]
l = len(P1)
index = 0
val = False

for i in range(l):
    if a[i][0]==b[0]:
        if a[i][1]==b[1]:
            index = i
            val = True
            break
        else:
            continue
    else:
        continue
    i = i + 1
Z = multiply (P1,P2)
Z.append(b)
M = Z[1] 
X = [0]*6
X[2] = 3.0
print("Matrix P1 is ",a)
print("Matrix P2 is ",P2)
print("Row is ",b)
print("Row belongs to matrix ",index, val)
print("Index is ",index)
print("Hadamard P1 and P2 is",Z)
print("2nd column elements of hadaamard are ",M)
print(max(M))
print(X)
