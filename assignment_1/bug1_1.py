#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from math import atan2, pi, cos, sin, sqrt

rospy.init_node('test', anonymous=True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# read input file
f = open("catkin_ws/src/sc627_assignments/assignment_1/input.txt", "r")
l = f.readline()
StartPoint = [float(l[0]), float(l[2])]
l = f.readline()
TargetPoint = [float(l[0]), float(l[2])]
l = f.readline()
StepSize = float(l)
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

Obstacle1 = [p1, p2, p3]
Obstacle2 = [p4, p5, p6]
Tolerance = 0.1 # Inputs
DirClock = 0 #Inputs (Anticlockwise Movement=0; Clockwise movement=1)

out = open("catkin_ws/src/sc627_assignments/assignment_2/output.txt", "w")
out.write("NEW WRITE")

# initialisation
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # radians
wp = MoveXYGoal()

V = computeVectorBetweenTwoPoints(StartPoint,TargetPoint); # init
Vstepped = StepSize*V #init
Path = StartPoint # init
currentPosition = StartPoint # init
Hit = 0 # init
TargetReach = 0 #init
Miss = 0
HuntEscape = 0
Circumnavigate = 0
Exists = 0

#path till obstacle is hit
while(TargetReach==0):
    candidateCurrentPosition = currentPosition + Vstepped
    [D1,wid1,v1,p1] = computeDistancePointToPolygon(Obstacle1,candidateCurrentPosition)
    [D2,wid2,v2,p2] = computeDistancePointToPolygon(Obstacle2,candidateCurrentPosition)
    
    if D1<D2:
        NearObstacle = Obstacle1
        DistObstacle = D1
    else:
        NearObstacle = Obstacle2
        DistObstacle = D2
    
    if HuntEscape == 0:
        if Circumnavigate == 0:
            if DistObstacle > Tolerance:
                currentPosition = candidateCurrentPosition
                Path.append(currentPosition)
                Hit = 0
            else:
                Miss = 1
                Hit = 1
                print("Obstacle Ahead. Starting Circumnavigation.")
                Circumnavigate = 1
        else: #logic for circumnavigation
            if Miss != 1:
                [Exists,Index] = computePointExistInArray(candidateCurrentPosition,Path)
                DistCircumStart = computeDistBetweenTwoPoints(Circumstart,candidateCurrentPosition)
                nn =len(Path)-2
                for i in range(nn):
                    Int(i)=doIntersect(currentPosition,candidateCurrentPosition,Path(i,:),Path(i+1,:));
                
                if ((Exists == 0) and (DistCircumStart >= StepSize-0.00001) and (max(Int) == 0)):
                    currentPosition = candidateCurrentPosition
                    Path.append(currentPosition)
                else:
                    Circumnavigate = 0
                    HuntEscape = 1
                    print("Obstacle Circumnavigation Complete. Looking for best escape route")
                    Miss=1
                    dd=[]
                end
            else:
                Miss=0
                Circumstart = currentPosition

            DistArray.append(computeDistBetweenTwoPoints(currentPosition,TargetPoint))
            [TVx,TVy] = computeTangentVectorToPolygon(NearObstacle,currentPosition,DirClock)
            A = [TVx, TVy]
            Vstepped = StepSize*A
    else: #logic to Hunt for Escape point
        if Miss !=1:
            currentPosition = candidateCurrentPosition
            Path.append(currentPosition)
            d = computeDistBetweenTwoPoints(currentPosition,TargetPoint)
            if d <= min(DistArray):
                HuntEscape = 0
                DistArray = []
                V = computeVectorBetweenTwoPoints(currentPosition,TargetPoint)
                Vstepped = StepSize*V
                Miss=1
            else:
                [TVx,TVy] = computeTangentVectorToPolygon(NearObstacle,currentPosition,DirClock)
                A = [TVx, TVy]
                Vstepped = StepSize*A
        else:
            Miss=0

    if computeDistBetweenTwoPoints(currentPosition,TargetPoint) <= StepSize:
        TargetReach = 1
        print ("Successfully reached target")
    else:
        TargetReach = 0

    wp.pose_dest.x = currentPosition[0]  # input x-point
    wp.pose_dest.y = currentPosition[1]  # input y-point
    wp.pose_dest.theta = theta = atan2(currentPosition[1], currentPosition[0])  # input theta point
    client.send_goal(wp)
    print("sent message")

    client.wait_for_result()
    print("received message")
    result = client.get_result()
    out.write("\n%s," % result.pose_final.x) #new line with x-coordinate and comma
    out.write("%s" % result.pose_final.y) # y-coordinate on same line

out.close()