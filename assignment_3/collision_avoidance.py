#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, atan2, sqrt, cos, sin
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import matplotlib.pyplot as plt


class Assign3:

    def __init__(self):
        
        # receive data
        rospy.Subscriber('/obs_data', ObsData, self.callback_obs)  # topic name fixed
        rospy.Subscriber('/bot_1/odom', Odometry, self.callback_odom)  # topic name fixed

        # send data
        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(30)

        self.ANG_MAX = pi*10/180    # 10 degrees to radians
        self.VEL_MAX = 0.15         # unit m/s
        self.TOL = 0.55              # tolerance for goal achieved
        self.radius = 0.15          # readius of robot given
        self.goal = [5, 0]          # goal coordinates given

        self.bot_pose = [0.0, 0.0]
        self.bot_v = [0.0, 0.0]
        self.bot_th = 0.0
        self.bot_omega = 0.0
        self.obs_pose_x = []
        self.obs_pose_y = []
        self.obs_v_x = []
        self.obs_v_y = []
        self.obs_radius=[]
        self.path_x = []
        self.path_y = []
        self.time = []
        self.start_time = time.time()

    def velocity_convert(self, x, y, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''
        gain_ang = 1  # modify if necessary

        ang = atan2(vel_y, vel_x)
        if ang < 0:
            ang += 2 * pi

        ang_err = min(max(ang - theta, -self.ANG_MAX), self.ANG_MAX)

        v_lin = min(max(cos(ang_err) * sqrt(vel_x ** 2 + vel_y ** 2), -self.VEL_MAX), self.VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.bot_pose[0] = data.pose.pose.position.x
        self.bot_pose[1] = data.pose.pose.position.y
        self.bot_th = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.path_x.append(self.bot_pose[0])
        self.path_y.append(self.bot_pose[1])
        self.time.append(time.time()-self.start_time)

    def callback_obs(self, data):
        '''
        Get left robot data
        '''
        self.numObs = len(data.obstacles)   # read no. of obstacles
        self.obs_pose_x = np.zeros(self.numObs)
        self.obs_pose_y = np.zeros(self.numObs) 
        self.obs_v_x = np.zeros(self.numObs)
        self.obs_v_y = np.zeros(self.numObs)
        self.obs_radius = np.zeros(self.numObs)
        for i in range(self.numObs):
            self.obs_pose_x[i] = data.obstacles[i].pose_x   # read each obstacles' position and store as list
            self.obs_pose_y[i] = data.obstacles[i].pose_y   # read each obstacles' position and store as list
            self.obs_v_x[i] = data.obstacles[i].vel_x       # read each obstacles' velocity and store as list
            self.obs_v_y[i] = data.obstacles[i].vel_y       # read each obstacles' position and store as list
            self.obs_radius[i] = 0.15

    def DistancePointToPoint(self, p1, p2):
        Distance = round((((p1[1]-p2[1])**2)+((p1[0]-p2[0])**2))**0.5, 4)
        return Distance

    def angleBetweenVector(self, a, b):
        v1 = np.array(a)
        v2 = np.array(b)
        angle = 0
        if np.linalg.norm(v1)*np.linalg.norm(v2) != 0.0:
            dot = np.dot((v1 / np.linalg.norm(v1)), (v2 / np.linalg.norm(v2)))
            if abs(dot) > 1:
                dot = np.sign(dot)*1
            angle = np.arccos(dot)
        return angle

    def allowedVelocities(self):  # returns set of allowable velocities which robot can take based on given velocity and angular constraints
        Allow_Orientations = np.arange((self.bot_th - self.ANG_MAX), (self.bot_th + self.ANG_MAX), pi/180) #equally spaced allowable velocity orientations for next step 1deg
        Allow_Magnitudes = np.arange(0, self.VEL_MAX, 0.01) #equally spaced allowable velocity magnitudes possible for next step 0.01m/s
        self.Allow_velocitySet = []
        for theta in Allow_Orientations:
            for v in Allow_Magnitudes:
                if v != 0:
                    allow_vel = [v*np.cos(theta), v*np.sin(theta)]
                    self.Allow_velocitySet.append(allow_vel)

    def velocityObstacle(self, obs_id):
        p1 = [self.bot_pose[0],self.bot_pose[1]]
        p2 = [self.obs_pose_x[obs_id],self.obs_pose_x[obs_id]]
        ratio = (self.radius + self.obs_radius[obs_id])/self.DistancePointToPoint(p1,p2)#self.computeDistanceTwoPoints(obs_id)
        if abs(ratio) > 1:
            ratio = np.sign(ratio)*1
        VOAngle = np.arcsin(ratio)
        return VOAngle

    def inVOCone(self, vel):
        numObs = len(self.obs_pose_x)
        x=False
        for i in range(numObs):
            resultant_vel = np.array([vel[0]-self.obs_v_x[i], vel[1]-self.obs_v_y[i]])
            robot_obs_line = np.array([self.obs_pose_x[i]-self.bot_pose[0], self.obs_pose_y[i]-self.bot_pose[1]])
            VelObsConeAngle = self.velocityObstacle(i)
            if self.angleBetweenVector(resultant_vel, robot_obs_line) < VelObsConeAngle:
                x=True
                break
        return x

    def noCollisionVelocities(self):
        self.noCollisionVel = []
        self.allowedVelocities()
        for vel in self.Allow_velocitySet:
            if not self.inVOCone(vel):
                self.noCollisionVel.append(vel)

    def TGheuristic(self): #TG TGheuristic selection of escape velocity
        self.noCollisionVelocities()
        alphaset = []
        robotToGoalVector = [self.goal[0]-self.bot_pose[0], self.goal[1]-self.bot_pose[1]]
        for velocity in self.noCollisionVel:
            alphaset.append(self.angleBetweenVector(robotToGoalVector, velocity))
        imin = np.argmin(alphaset)
        alphamin = alphaset[imin]
        VelFinal = self.noCollisionVel[imin] # to get the velocity vector with minimum deviation from robot to goal vector
        print('alpha=', alphamin)
        return VelFinal

if __name__ == '__main__':
    rospy.init_node('assign3_skeleton', anonymous=True)
    robot = Assign3()
    goalReached = False

    while not goalReached:
        cmd_x, cmd_y = robot.TGheuristic()
        v_lin, v_ang = robot.velocity_convert(robot.bot_pose[0], robot.bot_pose[1], robot.bot_th % (2*pi), cmd_x, cmd_y)
        P1=[robot.bot_pose[0],robot.bot_pose[1]]
        P2=robot.goal
        goalReached = robot.DistancePointToPoint(P1,P2) <= robot.TOL
        print(robot.bot_pose,robot.bot_th,cmd_x, cmd_y,'distance to goal=',robot.DistancePointToPoint(P1,P2))
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        robot.pub_vel.publish(vel_msg)
        robot.r.sleep()

        if rospy.is_shutdown() is True or goalReached is True:
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            robot.pub_vel.publish(vel_msg)
            robot.r.sleep()
            print("Goal Achieved")

            plt.figure(1)
            plt.plot(robot.path_x, robot.path_y)
            plt.ylabel('y-axis')
            plt.xlabel('x-axis')
            plt.title('Robot Path')
            plt.grid()

            plt.figure(2)
            plt.plot(robot.time, robot.path_x)
            plt.ylabel('Pos of robot')
            plt.xlabel('Time')
            plt.title('Pos vs. Time')
            plt.grid()

            plt.show()

            break

