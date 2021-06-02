#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2, isnan, sin, cos, acos
from explorer_turtle.msg import MarkerArray, Marker
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
import yaml
from marker_rviz import MarkerRviz
import tf

# CONSTANT

NUMBER_MARKERS = 8
Ts = 1/100 # control (u) by a translational and angular velocity, executed over a fixed time interval Ts

# Estimation error
sigma_range = 0.1
sigma_bearing = 0.1
sigma_id = 0.0

class EkfLocalization:

    def __init__(self):

        rospy.loginfo("locating...")

        with open("/home/alanpereira/catkin_ws/src/explorer_turtle/param/map.yaml", 'r') as stream:
            try:
                self.aruco_yaml = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
            
            stream.close()

        # self.save_date = open("/home/alanpereira/catkin_ws/src/explorer_turtle/docs/measurement.csv", "a")

        # important variables
        self.control = np.array([0, 0], dtype='float64').transpose()
        self.last_odom = np.array([0, 0, 0], dtype='float64').transpose()
        self.odom_init = False
        self.measurement = None
        self.markerStatus = False
        self.markerList = [False]*NUMBER_MARKERS

        # Choose very small process covariance because we are using the ground truth data for initial location
        self.sigma = np.diagflat([1e-10, 1e-10, 1e-10])
        self.H = 0
        self.Q = np.diagflat(np.array([sigma_range, sigma_bearing, sigma_id])) ** 2

        # Init Subscriber
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('/explorer/marker', Marker, self.marker_callback)
        #rospy.Subscriber('/explorer/markers', MarkerArray, self.marker_callback)

        # Init Publishers
        self.estimate_pub = rospy.Publisher('explorer/pose_filtered', Pose, queue_size=100)

        # list with rooms parameters 

        self.totem_0 = MarkerRviz("t0", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0))
        self.totem_1 = MarkerRviz("t1", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0)) 
        self.totem_2 = MarkerRviz("t2", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0)) 
        self.totem_3 = MarkerRviz("t3", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0))
        self.totem_4 = MarkerRviz("t4", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0)) 
        self.totem_5 = MarkerRviz("t5", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0)) 
        self.totem_6 = MarkerRviz("t6", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0))
        self.totem_7 = MarkerRviz("t7", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.1, 0.1, 1.0), (0.0, 1.0, 0.0)) 

        self.rate = rospy.Rate(100) # 100hz

    def run(self):
        while not rospy.is_shutdown():
            if self.odom_init and self.markerStatus:
                pos = self.last_odom
                control = self.control
                Z = self.measurement
                i = Z[2] # Feature
                self.prediction(pos, control)
                self.sigma = self.G.dot(self.sigma).dot(self.G.T) + self.R
                ZHAT = self.aruco_true(self.aruco_yaml[i], i, pos)

                residual = np.subtract(Z, ZHAT)

                S = self.H.dot(self.sigma).dot(self.H.T) + self.Q
                if np.linalg.det(S):
                    K = self.sigma.dot(self.H.T).dot(np.linalg.inv(S))
                else:
                    K = np.array([0, 0, 0], dtype='float64')
                mu = pos + np.dot(K, residual)
                self.sigma = (np.identity(3) - np.cross(K, self.H))*self.sigma
                self.pub_position(mu, self.sigma)
                self.markerVisualization()
                self.resetMarkers()

            self.rate.sleep()

    def markerVisualization(self):

        if self.markerList[0] == True:
            self.totem_0.publish()
        if self.markerList[1] == True:
            self.totem_1.publish()
        if self.markerList[2] == True:
            self.totem_2.publish()
        if self.markerList[3] == True:
            self.totem_3.publish()
        if self.markerList[4] == True:
            self.totem_4.publish()
        if self.markerList[5] == True:
            self.totem_5.publish()
        if self.markerList[6] == True:
            self.totem_6.publish()
        if self.markerList[7] == True:
            self.totem_7.publish()

        br_room_0 = tf.TransformBroadcaster()
        br_room_0.sendTransform(self.aruco_yaml[0], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t0", "map")

        br_room_1 = tf.TransformBroadcaster()
        br_room_1.sendTransform(self.aruco_yaml[1], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t1", "map")

        br_room_2 = tf.TransformBroadcaster()
        br_room_2.sendTransform(self.aruco_yaml[2], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t2", "map")

        br_room_3 = tf.TransformBroadcaster()
        br_room_3.sendTransform(self.aruco_yaml[3], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t3", "map")

        br_room_4 = tf.TransformBroadcaster()
        br_room_4.sendTransform(self.aruco_yaml[4], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t4", "map")

        br_room_5 = tf.TransformBroadcaster()
        br_room_5.sendTransform(self.aruco_yaml[5], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t5", "map")

        br_room_6 = tf.TransformBroadcaster()
        br_room_6.sendTransform(self.aruco_yaml[6], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t6", "map")

        br_room_7 = tf.TransformBroadcaster()
        br_room_7.sendTransform(self.aruco_yaml[7], quaternion_from_euler(0.0, 0.0, 0.0), rospy.Time.now(), "t7", "map")

    def resetMarkers(self):

        self.measurement = None
        self.markerStatus = False

    def pub_position(self, mu, sigma):
        
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        quat = quaternion_from_euler(0, 0, mu[2])

        estimate = Pose()

        # set the position
        estimate = Pose(Point(mu[0], mu[1], 0.), Quaternion(*quat))

        # publish the message
        self.estimate_pub.publish(estimate)

    def prediction(self, pos, control):

        # compute the Jacobians needed for the linearized motion model.
        th = pos[2]
        v = control[0]
        w = control[1]

        G_02 = -v*Ts*sin(th + (w*Ts)/2)
        G_12 = v*Ts*cos(th + (w*Ts)/2)

        self.G = np.identity(3, dtype='float64')
        self.G[0][2] = G_02
        self.G[1][2] = G_12

    def odom_callback(self, msg):

        # get x, y and yaw
        q = msg.pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        # the yaw is defined between -pi to pi
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        current_odom = np.array([x, y, yaw], dtype='float64').transpose()

        if not self.odom_init:
            self.last_odom = np.array([x, y, yaw], dtype='float64').transpose()
            self.odom_init = True

        # Get control vector (change in linear displacement and rotation)to estimate current pose of the robot
        self.last_odom = current_odom
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.control = np.array([v, w], dtype='float64').transpose()

        # UNCERTAINTY INTRODUCED BY STATE TRANSITION (MEAN = 0, COVARIANCE PUBLISHED BY ODOM TOPIC: )
        # Odom covariance matrix is 6 x 6. We need a 3 x 3 covariance matrix of x, y and theta. Omit z, roll and pitch data. 
        self.R = np.array([[msg.pose.covariance[0],msg.pose.covariance[1],msg.pose.covariance[5]],
        [msg.pose.covariance[6],msg.pose.covariance[7],msg.pose.covariance[11]], [msg.pose.covariance[30],msg.pose.covariance[31],msg.pose.covariance[35]]])

    def marker_callback(self, msg):

        try:
            index = msg.id
            if(index >=0 and index < NUMBER_MARKERS): # aruco tag id from 0 to NUMBER_MARKERS
                dist = msg.distance
                th = msg.angle
                self.measurement = np.array([dist, th , index]).transpose()
                self.markerStatus = True
                self.markerList[index] = True
        except:
            pass


    def aruco_true(self, aruco, index, robot):

        lx = float(aruco[0])
        ly = float(aruco[1])

        rx = robot[0]
        ry = robot[1]
        rt = robot[2]

        q = (lx - rx)**2 + (ly - ry)**2 + 0.01
        dist = sqrt(q)
        th = atan2((ly-ry),(lx-rx) ) - rt

        self.H = np.array([[-(lx - rx)/dist, -(ly - ry)/dist , 0],[(ly - ry)/q, -(lx - rx)/q, -1],[0 , 0, 0]])

        Z = np.array([dist, th, index]).transpose()

        return Z

if __name__ == '__main__':
    try:
        rospy.init_node("ekf_localization")
        e = EkfLocalization()
        e.run()
        
    except rospy.ROSInterruptException:
        pass