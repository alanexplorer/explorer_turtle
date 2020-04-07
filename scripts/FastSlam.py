#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2, isnan
from aruco_msgs.msg import MarkerArray
from ass.particles import Particle
import random
from copy import deepcopy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped

# Costante

NUMBER_PARTICLES = 100
NUMBER_MARKERS = 4

fig, ax = plt.subplots()
robot_odom, = ax.plot([0], [0], color='black', marker='o', markersize=12)
robot_estimate, = ax.plot([0], [0], color='red', marker='*', markersize=12)
plt.ion()
plt.xlim(-10, 20)
plt.ylim(-20, 10)
plt.xlabel('X', fontsize=10)  # X axis label
plt.ylabel('Y', fontsize=10)  # Y axis label
plt.title('FastSlam')
#plt.legend()
plt.grid(True)  # Enabling gridding


class FastSLAM:

	def __init__(self):

		# important variables
		self.odom_data = np.array([0, 0, 0], dtype='float64').transpose()
		self.state_trans_uncertainty_noise = np.eye(3, dtype=int)*1000
		self.odom_init = False
		self.marker_data = None
		self.id = float("NaN")
		self.x_path = np.array([0], dtype='float64')
		self.y_path = np.array([0], dtype='float64')

		self.particles = [Particle(NUMBER_PARTICLES) for i in range(NUMBER_PARTICLES)]
		self.aruco_list = [None]*NUMBER_MARKERS

		rospy.Subscriber('odom', Odometry, self.odom_callback)
		rospy.Subscriber('aruco_marker_publisher/markers', MarkerArray, self.marker_callback)

		self.new_pose = Pose()

        # Init publishers
		self.estimate_pub = rospy.Publisher('explorer/pose_filtered', Pose, queue_size=10)

		self.rate = rospy.Rate(100) # 100hz

	def run(self):

		while not rospy.is_shutdown():
			if self.odom_init and not isnan(self.id) :
				total_weight = 0
				for i in range(NUMBER_PARTICLES):
					# set a position for each particles
					self.particles[i].set_position(self.odom_data)
					self.particles[i].predicted(self.state_trans_uncertainty_noise)
					ave_meas_dist = self.landmarker_measured(self.marker_data)
					self.particles[i].update(ave_meas_dist)
					self.particles[i].update_weight()
					total_weight += self.particles[i].get_weight()
				
				for i in range(NUMBER_PARTICLES):
					self.particles[i].normalize_weight(total_weight)

				self.particles = self.resample_particles(self.particles)
				#self.drawing_plot(self.particles)
				self.pub_est(self.particles)

			self.id = float("NaN")
			self.rate.sleep()

	def pub_est(self, particles):

		Max = 0
		Max_id = 0

		for i in range(NUMBER_PARTICLES):
			if particles[i].get_weight() > Max:
				Max = particles[i].get_weight()
				Max_id = i

		estimative = particles[Max_id].get_position()
		
        # pack up estimate to ROS msg and publish

		self.new_pose.position.x = estimative[0]
		self.new_pose.position.y = estimative[1]
		self.new_pose.position.z = 0

		quat = quaternion_from_euler(0, 0, estimative[2])

		self.new_pose.orientation.x = quat[0]
		self.new_pose.orientation.y = quat[1]
		self.new_pose.orientation.z = quat[2]
		self.new_pose.orientation.w = quat[3]

		self.estimate_pub.publish(self.new_pose)

	def landmarker_measured(self, marker):

		#Equation 3.35  - FastSLAM: A factored solution to the simultaneous localization and mapping problem

		th_y = marker.pose.pose.position.y # right-left 
		th_x = marker.pose.pose.position.z # front-back 

		# position of the marker relative to camera_link

		distance = sqrt(th_x**2 + th_y**2)

		return distance

	def odom_callback(self, msg):

		# get x, y and yaw
		q = msg.pose.pose.orientation
		orientation_list = [q.x, q.y, q.z, q.w]
		# the yaw is defined between -pi to pi
		(_, _, yaw) = euler_from_quaternion(orientation_list)
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y

		self.odom_data = np.array([x, y, yaw], dtype='float64').transpose()

		# UNCERTAINTY INTRODUCED BY STATE TRANSITION (MEAN = 0, COVARIANCE PUBLISHED BY ODOM TOPIC: )
		# Odom covariance matrix is 6 x 6. We need a 3 x 3 covariance matrix of x, y and theta. Omit z, roll and pitch data. 
		self.state_trans_uncertainty_noise = np.array([[msg.pose.covariance[0],msg.pose.covariance[1],msg.pose.covariance[5]],
		[msg.pose.covariance[6],msg.pose.covariance[7],msg.pose.covariance[11]], [msg.pose.covariance[30],msg.pose.covariance[31],msg.pose.covariance[35]]])

		self.x_path = np.insert(self.x_path, 0, x)
		self.y_path = np.insert(self.y_path, 0, y)

		self.odom_init = True
	
	def marker_callback(self, msg):
		try:
			self.marker_data = msg.markers.pop()
			self.id = self.marker_data.id

		except rospy.ROSInterruptException:
			pass

	def resample_particles(self, particles):
		new_particles = []
		weight = [p.get_weight() for p in particles]
		index = int(random.random() * NUMBER_PARTICLES)
		beta = 0.0
		mw = max(weight)
		for i in range(NUMBER_PARTICLES):
			beta += random.random() * 2.0 * mw
			while beta > weight[index]:
				beta -= weight[index]
				index = (index + 1) % NUMBER_PARTICLES
			new_particle = deepcopy(particles[index])
			new_particle.weight = 1
			new_particles.append(new_particle)
		return new_particles

	def drawing_plot(self, particles):

		Max = 0
		Max_id = 0

		for i in range(NUMBER_PARTICLES):
			if particles[i].get_weight() > Max:
				Max = particles[i].get_weight()
				Max_id = i
		
		plt.show(block=False)

		robot_odom.set_xdata(self.odom_data[0])
		robot_odom.set_ydata(self.odom_data[1])
		ax.draw_artist(ax.patch)
		ax.draw_artist(robot_odom)

		estimative = particles[Max_id].get_position()

		robot_estimate.set_xdata(estimative[0])
		robot_estimate.set_ydata(estimative[1])
		ax.draw_artist(ax.patch)
		ax.draw_artist(robot_estimate)

		fig.canvas.flush_events()

	def get_path(self):
		return self.x_path, self.y_path


if __name__ == '__main__':
    try:
        rospy.init_node("FastSLAM")
        f = FastSLAM()
        f.run()
        
    except rospy.ROSInterruptException:
        pass