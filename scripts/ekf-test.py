#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import tf
from math import sin, cos, tan, atan2, sqrt
from aruco_msgs.msg import MarkerArray

class ekf_slam():
    def __init__(self):

        # Estimator stuff
        # x = pn, pe, pd, phi, theta, psi
        self.xhat = np.zeros((9,1))
        self.xhat_odom = Odometry()

        # Covariance matrix
        self.P = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.Q = np.diag([2.0, 1.0]) # meas noise

        # Measurements stuff
        # Truth
        self.truth_pn = 0.0
        self.truth_pe = 0.0
        self.truth_pd = 0.0
        self.truth_phi = 0.0
        self.truth_theta = 0.0
        self.truth_psi = 0.0
        self.truth_p = 0.0
        self.truth_q = 0.0
        self.truth_r = 0.0
        self.truth_pndot = 0.0
        self.truth_pedot = 0.0
        self.truth_pddot = 0.0
        self.truth_u = 0.0
        self.truth_v = 0.0
        self.truth_w = 0.0
        self.prev_time = 0.0
        self.imu_az = 0.0

        #aruco Stuff
        self.aruco_location = {
        100:[0.0, 0.0, 0.0],
        101:[0.0, 14.5, 5.0],
        102:[5.0, 14.5, 5.0],
        103:[-5.0, 14.5, 5.0],
        104:[0.0, -14.5, 5.0],
        105:[5.0, -14.5, 5.0],
        106:[-5.0, -14.5, 5.0],
        107:[7.0, 0.0, 5.0],
        108:[7.0, 7.5, 5.0],
        109:[7.0, -7.5, 5.0],
        110:[-7.0, 0.0, 5.0],
        111:[-7.0, 7.5, 5.0],
        112:[-7.0, -7.5, 5.0],
        }

        # Number of propagate steps
        self.N = 1

        #Constants
        self.g = 9.8

        # ROS Stuff
        # Init subscribers
        self.odom_sub_ = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #self.imu_sub_ = rospy.Subscriber('/slammer/imu/data', Imu, self.imu_callback)
        self.aruco_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback )

        # Init publishers
        #self.estimate_pub_ = rospy.Publisher('/ekf_estimate', Odometry, queue_size=10)

        # # Init Timer
        #self.pub_rate_ = 100. #
        #self.update_timer_ = rospy.Timer(rospy.Duration(1.0/self.pub_rate_), self.pub_est)

    def odom_callback(self, msg):

        self.truth_pn = msg.pose.pose.position.x
        self.truth_pe = msg.pose.pose.position.y
        self.truth_pd = msg.pose.pose.position.z

        quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)

        self.truth_phi = euler[0]
        self.truth_theta = euler[1]
        self.truth_psi = euler[2]

        self.truth_p = msg.twist.twist.angular.x
        self.truth_q = msg.twist.twist.angular.y
        self.truth_r = msg.twist.twist.angular.z

        self.truth_u = msg.twist.twist.linear.x
        self.truth_v = msg.twist.twist.linear.y
        self.truth_w = msg.twist.twist.linear.z


    def aruco_callback(self, msg):
        
        if len(msg.markers) > 0:
            for i in range(0,len(msg.markers)):

                self.aruco_id = msg.markers[i].id
                self.aruco_x = msg.markers[i].pose.pose.position.x
                self.aruco_y = msg.markers[i].pose.pose.position.y
                self.aruco_z = msg.markers[i].pose.pose.position.z

                quat = (
                    msg.markers[i].pose.pose.orientation.x, 
                    msg.markers[i].pose.pose.orientation.y, 
                    msg.markers[i].pose.pose.orientation.z, 
                    msg.markers[i].pose.pose.orientation.w)

                euler = tf.transformations.euler_from_quaternion(quat)
                
                self.aruco_phi = euler[0]
                self.aruco_theta = euler[1]
                self.aruco_psi = euler[2]

                self.range = np.sqrt(self.aruco_x**2 + self.aruco_y**2 + self.aruco_z**2)
                
                self.bearing_2d = np.arctan2(self.aruco_x, self.aruco_z)#-self.truth_psi
                self.z = np.array([[self.range],[self.bearing_2d]])

                self.update()

    def update(self):

        m = self.aruco_location[self.aruco_id]

        rangehat = np.double(np.sqrt((m[0]-self.xhat[0])**2 + (-m[1]-self.xhat[1])**2))

        zhat = np.array([[rangehat],
                        [np.arctan2(-m[1]-self.xhat[1],m[0]-self.xhat[0])-self.xhat[8]]])

        C = np.array([[np.double(-(m[0]-self.xhat[0])/rangehat) , -((-m[1])-self.xhat[1])/rangehat,0,0,0,0,0,0,0 ],
                           [((-m[1])-self.xhat[1])/rangehat**2  , -(m[0]-self.xhat[0])/rangehat**2,0,0,0,0,0,0,-1]])

        # print self.aruco_id, self.xhat[0], self.xhat[1], self.range, self.H

        S = np.matmul(C,np.matmul(self.P,C.T))+self.Q

        self.L =np.matmul(self.P,np.matmul(C.T,np.linalg.inv(S)))

        # wrap the residual
        residual = self.z-zhat
        if residual[1] > np.pi:
            residual[1] -= 2*np.pi
        if residual[1] < -np.pi:
            residual[1] += 2*np.pi
        print("zhat", zhat)
        print("Residua", residual)

        dist = residual.T.dot(np.linalg.inv(S)).dot(residual)[0,0]
        # print "dist", dist
        if dist < 9:
            self.xhat = self.xhat + np.matmul(self.L,(residual))
            self.P = np.matmul((np.identity(9)-np.matmul(self.L,C)),self.P)
        else:
            print("gated a measurement", np.sqrt(dist))

    def pub_est(self, event):

        # pack up estimate to ROS msg and publish
        self.xhat_odom.header.stamp = rospy.Time.now()
        self.xhat_odom.pose.pose.position.x = self.xhat[0] # pn
        self.xhat_odom.pose.pose.position.y = self.xhat[1] # pe
        self.xhat_odom.pose.pose.position.z = self.xhat[2] # pd

        quat = tf.transformations.quaternion_from_euler(self.xhat[6].copy(), self.xhat[7].copy(), self.xhat[8].copy())

        self.xhat_odom.pose.pose.orientation.x = quat[0]
        self.xhat_odom.pose.pose.orientation.y = quat[1]
        self.xhat_odom.pose.pose.orientation.z = quat[2]
        self.xhat_odom.pose.pose.orientation.w = quat[3]
        self.xhat_odom.twist.twist.linear.x = self.xhat[3] # u
        self.xhat_odom.twist.twist.linear.y = self.xhat[4] # v
        self.xhat_odom.twist.twist.linear.z = self.xhat[5] # w

        self.estimate_pub_.publish(self.xhat_odom)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__': 

    rospy.init_node('slam_estimator')

    estimator = ekf_slam()
    estimator.run()

    while not rospy.is_shutdown():
        rospy.spin()