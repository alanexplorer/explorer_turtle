#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

# PRIOR KNOWLEDGE OF STATE
predicted_state_est = 0
predicted_covariance_est = 0
state_trans_uncertainty_noise = 0
measurement =	0
ave_meas_dist = 0

def odom_state_prediction(odom_data):

    global predicted_state_est
    (roll,pitch,yaw) = euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y

    state_trans_uncertainty_noise = np.array([[odom_data.pose.covariance[0],odom_data.pose.covariance[1],odom_data.pose.covariance[5], [odom_data.pose.covariance[6],odom_data.pose.covariance[7],odom_data.pose.covariance[11]], [odom_data.pose.covariance[30],odom_data.pose.covariance[31],odom_data.pose.covariance[35]]]])
     
    state_transition_jacobian = np.array([[1,0,0],[0,1,0],[0,0,1]])

    predicted_covariance_est = state_transition_jacobian*predicted_covariance_est*np.transpose(state_transition_jacobian) + state_trans_uncertainty_noise
    
    print(predicted_covariance_est)

	
if __name__ == "__main__":
    rospy.init_node('oodometry', anonymous=True)

    rospy.Subscriber('odom', Odometry, odom_state_prediction)

    rospy.spin()