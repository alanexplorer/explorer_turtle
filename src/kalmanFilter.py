#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi,tan, atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np

class efk:

    def __init__(self):

        self.predicted_state_est = 0
        self.predicted_covariance_est = 0
        self.state_trans_uncertainty_noise = 0
        self.measurement =	0
        self.ave_meas_dist = 0
        
        # define subscribers
        #rospy.Subscriber('laser', LaserScan, self.scan_callback)
        rospy.Subscriber('odom', Odometry, self.odom_state_prediction)

        #publish rate
        self.rate = rospy.Rate(10) # 500hz   


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def odom_state_prediction(self, odom_data):

        global predicted_state_est
        (roll,pitch,yaw) = euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y

        self.state_trans_uncertainty_noise = np.array([[odom_data.pose.covariance[0],odom_data.pose.covariance[1],odom_data.pose.covariance[5], [odom_data.pose.covariance[6],odom_data.pose.covariance[7],odom_data.pose.covariance[11]], [odom_data.pose.covariance[30],odom_data.pose.covariance[31],odom_data.pose.covariance[35]]]])
        
        state_transition_jacobian = np.array([[1,0,0],[0,1,0],[0,0,1]])

        self.predicted_covariance_est = state_transition_jacobian*self.predicted_covariance_est*np.transpose(state_transition_jacobian) + self.state_trans_uncertainty_noise
        
        print(self.predicted_covariance_est)

if __name__ == '__main__':
    try:
        rospy.init_node("kalmanFilter")
        m = efk()
        m.run()
        
    except rospy.ROSInterruptException:
        pass