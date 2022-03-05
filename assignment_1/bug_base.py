#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from math import atan2, pi, cos, sin
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


k = 1
r = [[0, 0], [0, 0]]


# open output file
out = open("catkin_ws/src/sc627_assignments/assignment_1/output_base.txt", "a")

# setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # in radians (0 to 2pi)
wp = MoveXYGoal()


# Bug_base Algorithm
while DistancePointToPoint(r[k], goal) > step:
    print("Start")
    [Vx, Vy, th] = VectorToPoint(r[k], goal)
    newx = float(r[k][0]+step*Vx)
    newy = float(r[k][1]+step*Vy)

    a = computeDistancePointToPolygon(r[k], P1)[0]
    b = computeDistancePointToPolygon(r[k], P2)[0]
    if a < b:
        Ob = P1
    else:
        Ob = P2

    # if obstacle deteced
    if computeDistancePointToPolygon([newx, newy], Ob)[0] < step:
        print("Failure: There is an obstacle lying between the start and goal")
        out.write(
            "\nFailure: There is an obstacle lying between the start and goal")
        break
    else:
        # r.append([newx, newy])
        print("else case")
        wp.pose_dest.x = newx
        wp.pose_dest.y = newy
        wp.pose_dest.theta = th
        client.send_goal(wp)
        print("sent message")

        client.wait_for_result()
        print("received message")
        result = client.get_result()
        r.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k += 1
if DistancePointToPoint(r[k], goal) < step:
    print("Success: Reached the goal")
    out.write("\nSuccess: Reached the goal")
print(r)

out.close()
