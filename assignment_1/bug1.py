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
out = open("catkin_ws/src/sc627_assignments/assignment_1/output_bug1.txt", "a")

# setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # in radians (0 to 2pi)
wp = MoveXYGoal()


# Bug1 Algorithm
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
        [V1, V2, th] = (computeTangentVectorToPolygon(r[k], Ob))
        newx1 = float(r[k][0]+step*V1)
        newy1 = float(r[k][1]+step*V2)
        # r.append([newx1, newy1])
        wp.pose_dest.x = newx1
        wp.pose_dest.y = newy1
        wp.pose_dest.theta = th
        client.send_goal(wp)

        client.wait_for_result()
        result = client.get_result()
        r.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k = k+1
        hit = k
        print("r hit", r[hit])
        [V1, V2, th] = (computeTangentVectorToPolygon(r[k], Ob))
        newx1 = float(r[k][0]+step*V1)
        newy1 = float(r[k][1]+step*V2)
        # r.append([newx1, newy1])
        # print("rk1", r[k], k)
        wp.pose_dest.x = newx1
        wp.pose_dest.y = newy1
        wp.pose_dest.theta = th
        client.send_goal(wp)

        client.wait_for_result()
        result = client.get_result()
        r.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k = k+1
        [V1, V2, th] = (computeTangentVectorToPolygon(r[k], Ob))
        newx1 = float(r[k][0]+step*V1)
        newy1 = float(r[k][1]+step*V2)
        # r.append([newx1, newy1])
        # print("rk1", r[k], k)
        wp.pose_dest.x = newx1
        wp.pose_dest.y = newy1
        wp.pose_dest.theta = th
        client.send_goal(wp)

        client.wait_for_result()
        result = client.get_result()
        r.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k = k+1

        # while the original hit point is not acheived
        while not DistancePointToPoint(r[k], r[hit]) < step:
            if computeDistancePointToPolygon(r[k], Ob)[0] > 2*step:
                T = computeDistancePointToPolygon(r[k], Ob)[2]
                [V1, V2, th] = VectorToPoint(r[k], T)
                # print(r[k])
                # print("TTTTTTTTTTTTTTTTT",T)
            else:
                [V1, V2, th] = (computeTangentVectorToPolygon(r[k], Ob))
                T = computeDistancePointToPolygon(r[k], Ob)[2]
                # print("T",T)
            # print("v",V1,V2)
            newx1 = float(r[k][0]+step*V1)
            newy1 = float(r[k][1]+step*V2)
            # r.append([newx1, newy1])
            wp.pose_dest.x = newx1
            wp.pose_dest.y = newy1
            wp.pose_dest.theta = th

            client.send_goal(wp)
            print("send1")

            client.wait_for_result()
            print("rec1")
            result = client.get_result()
            r.append([result.pose_final.x, result.pose_final.y])
            out.write("\n% s," % result.pose_final.x)
            out.write("%s" % result.pose_final.y)
            k = k+1
        # print(r)
        print("out", r[k])
        i = hit+2
        minx = r[hit][0]
        miny = r[hit][1]
        print(k)
        while i < k:
            if DistancePointToPoint(r[i], goal) < DistancePointToPoint([minx, miny], goal):
                minx = r[i][0]
                miny = r[i][1]

            i = i+1
        print("min", minx, miny)
        pleave = [minx, miny]
        while not DistancePointToPoint(r[k], pleave) < step:
            # print("in")
            if computeDistancePointToPolygon(r[k], Ob)[0] > 2*step:
                T = computeDistancePointToPolygon(r[k], Ob)[2]
                [V1, V2, th] = VectorToPoint(r[k], T)
                # print(r[k])
                # print("TTTTTTTTTTTTTTTTT",T)
            else:
                [V1, V2, th] = (computeTangentVectorToPolygon(r[k], Ob))
                T = computeDistancePointToPolygon(r[k], Ob)[2]
            newx1 = float(r[k][0]+step*V1)
            newy1 = float(r[k][1]+step*V2)
            # r.append([newx1, newy1])
            wp.pose_dest.x = newx1
            wp.pose_dest.y = newy1
            wp.pose_dest.theta = th
            client.send_goal(wp)

            client.wait_for_result()
            result = client.get_result()
            r.append([result.pose_final.x, result.pose_final.y])
            out.write("\n% s," % result.pose_final.x)
            out.write("%s" % result.pose_final.y)

            k = k+1
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

print(r)

out.close()
