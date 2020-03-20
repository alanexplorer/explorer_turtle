#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import std_msgs.msg
from explorer_turtle.msg import * 
import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt

predicted_state_est = 0
predicted_covariance_est = 0
state_trans_uncertainty_noise = 0
measurement =	0
ave_meas_dist = 0

pub = rospy.Publisher('state_estimate',Config, queue_size =10)
def get_data():

    rospy.init_node('localization', anonymous=True)
    # define ros param
    laser = rospy.get_param("/laser_topic")
    odom = rospy.get_param("/odometry_topic")

    rospy.Subscriber(laser, LaserScan, kinect_scan_estimate)

    rospy.Subscriber(odom, Odometry, odom_state_prediction)

	# The callback is scheduled for every 1/1127 th of a second (8.8hz), should be ajusted

    rospy.Timer(rospy.Duration(0.1127), meas_update_step, oneshot=False)

    rospy.spin()
    
def kinect_scan_estimate(msg):
    
    global measurement
    global ave_meas_dist

    measurement = msg.ranges
    
    sum_dist = 0
    length = 0
	
    for i in range (580, 600):
        if str(measurement[i]) != 'nan' :
            sum_dist += measurement[i]
            length += 1
            
    if length != 0:
        ave_meas_dist = sum_dist/length  


def odom_state_prediction(msg):

    global predicted_state_est
    global state_trans_uncertainty_noise
    global predicted_covariance_est

    (roll,pitch,yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    predicted_state_est = Config(x,y,yaw)

    state_trans_uncertainty_noise= np.array([[msg.pose.covariance[0],msg.pose.covariance[1],msg.pose.covariance[5]],[msg.pose.covariance[6],msg.pose.covariance[7],msg.pose.covariance[11]], [msg.pose.covariance[30],msg.pose.covariance[31],msg.pose.covariance[35]]])

    state_transition_jacobian = np.array([[1,0,0],[0,1,0],[0,0,1]])

    predicted_covariance_est = state_transition_jacobian*predicted_covariance_est*np.transpose(state_transition_jacobian)+state_trans_uncertainty_noise

def meas_update_step(event):

    global pub
    global predicted_covariance_est

    expected_meas = np.cross(np.array([0, 1, 0]), np.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]))

    meas_residual = ave_meas_dist - expected_meas

    meas_noise_covariance = 0.005

    H = np.array([[9999, 0 , 0],[0, 1, 0],[0 , 0, 9999]])

    residual_covariance = H*predicted_covariance_est*np.transpose(H)+meas_noise_covariance

    kalman_gain = predicted_covariance_est*np.transpose(H)*np.linalg.inv(residual_covariance)

    updated_state_estimate =  np.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]) + np.dot(kalman_gain, meas_residual)

    predicted_covariance_est= (np.identity(3) - np.cross(kalman_gain,H))*predicted_covariance_est

    state_estimate = Config(updated_state_estimate[0], updated_state_estimate[1], updated_state_estimate[2])

    rospy.logdebug(state_estimate)

    print(state_estimate.x, state_estimate.y, state_estimate.th)

    pub.publish(state_estimate)

if __name__ == '__main__':
    try:
        get_data()
        
    except rospy.ROSInterruptException:
        pass