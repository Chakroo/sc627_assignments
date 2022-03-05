#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from math import atan2, pi, cos, sin, sqrt
from helper import *

rospy.init_node('test', anonymous=True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# read input file
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

out = open("catkin_ws/src/sc627_assignments/assignment_2/output.txt", "a")
out.write("NEW WRITE")

# initialisation
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # radians
wp = MoveXYGoal()
i = 1
r = [[0, 0], [0, 0]]  # start point
dstargoal = 2  # given in assignment writeup
X = 0.8  # given in assignment writeup
Qstari = 2  # given in assignment writeup
ne = 0.8  # given in assignment writeup


# Potential Function
while DistancePointToPoint(r[i], goal) > step:
    print("Started movement")
    [Ax, Ay] = attPotentialGrad(r[i], goal, dstargoal, X)
    [R1x, R1y] = repPotentialGrad(r[i], P1, Qstari, ne)
    [R2x, R2y] = repPotentialGrad(r[i], P2, Qstari, ne)

    # for making robot work in gazebo environment
    [Vx, Vy, theta] = normalisation([Ax+R1x+R2x, Ay+R1y+R2y])

    nextx = float(r[i][0]+step*Vx)
    nexty = float(r[i][1]+step*Vy)

    wp.pose_dest.x = nextx  # input x-point
    wp.pose_dest.y = nexty  # input y-point
    wp.pose_dest.theta = theta  # input theta point
    client.send_goal(wp)
    print("sent message")

    client.wait_for_result()
    print("received message")
    result = client.get_result()
    r.append([result.pose_final.x, result.pose_final.y])
    out.write("\n%s," % result.pose_final.x) #new line with x-coordinate and comma
    out.write("%s" % result.pose_final.y) # y-coordinate on same line
    i = i + 1

    # if attractive gradient goes to avalue lesser than some very low value, the movement should stop - minima
    if ((sqrt((Ax**2)+(Ay**2)))) < 0.2:
        break

print(r)

out.close()
