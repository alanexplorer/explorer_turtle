#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from aruco_msgs.msg import MarkerArray
import numpy as np
import matplotlib.pyplot as plt
import time
import copy
from Class.particle import Particle


#Ponto de partida (0,0) - Porta da sala
#Marker de id 0 tem coordenadas (x_map0,y_map0) em metros

x_map=[0.0]*4
y_map=[0.0]*4

x_map[0]=3.497980
y_map[0]=-0.096514
x_map[1]= -0.021379
y_map[1]= 3.164220
x_map[2]= -3.497550
y_map[2]= 0.030892
x_map[3]= -0.018278
y_map[3]= -3.858117


camara_distance_z = 0.12 # 15.5 cm  <-> 13 cm #dia 13/12/2018 <-> 12 cm => 12.5 cm   inicio a 81 cm
camara_distance_x = 0.011 # 1.1 cm

# Constants

KEY_NUMBER = 2**(5*5) # number of total combinations possible in aruco code
number_of_dimensions = 2
Frequency = 9.5

NUMBER_PARTICLES = 100
NUMBER_MARKERS = 4
translation_noise = 0.1
rotation_noise = 0.1
noise_factor = 1
minimum_move = 0
Sensor_noise = 0.1
validity_threshold = 50

def resample_particles(particles, updated_marker):

	# Returns a new set of particles obtained by performing stochastic universal sampling, according to the particle weights.
	# distance between pointers
	step = 1.0/NUMBER_PARTICLES
	# random start of first pointer
	r = np.random.uniform(0,step)
	# where we are along the weights
	c = particles[0].get_weight()
	# index of weight container and corresponding particle
	i = 0
	index = 0

	new_particles = []

	#loop over all particle weights
	for _ in particles:
		#go through the weights until you find the particle
		u = r + index*step
		while u > c:
			i = i + 1
			c = c + particles[i].get_weight()

		#add that particle
		if i == index:
			new_particle = particles[i]
			new_particle.set_weight(step)
		else:
			new_particle = particles[i].copy(updated_marker)
		#new_particle = copy.deepcopy(particles[i])
		#new_particle.set_weight(step)
		new_particles.append(new_particle)
		#increase the threshold

		index += 1

	del particles

	return new_particles


class FastSlam():

	def __init__(self):

		self.read_move = np.array([0, 0, 0], dtype='float64').transpose()
		self.first_read = True
		self.particles = [Particle() for i in range(NUMBER_PARTICLES)]
		self.updated_marker = [False]*NUMBER_MARKERS

		self.could_it_read = False
		self.z_distance_left_eye_to_robot_wheel = camara_distance_z
		self.x_distance_left_eye_to_robot_wheel = camara_distance_x
		self.markers_info = [None]*NUMBER_MARKERS
		self.list_ids = np.ones(NUMBER_MARKERS, dtype='int32')*KEY_NUMBER

		rospy.init_node('FastSlam', anonymous=True)

		self.frequency = rospy.Rate(Frequency)

		rospy.Subscriber("odom", Odometry, self.callback_odom)

		rospy.Subscriber('aruco_marker_publisher/markers', MarkerArray, self.callback_Markers)

		self.posePublisher = rospy.Publisher('explorer/pose_filtered', Pose, queue_size=1)

	def run(self):

		while not rospy.is_shutdown():

			motion_model = self.actual_movement()
			landmarkers_ids = self.get_list_ids()
			if landmarkers_ids[0] != KEY_NUMBER and np.linalg.norm(motion_model) > minimum_move:
				total_weight = 0
				motion_model = self.get_movement()
				
				for i in range(NUMBER_PARTICLES):
					expected_state = self.particles[i].particle_prediction(motion_model)
					for marker_id in landmarkers_ids:
						if marker_id == KEY_NUMBER:
							break
						self.updated_marker[marker_id] = True
						landmarker_measurement = self.get_measerment(marker_id)
						kalman_filter = self.particles[i].get_kalman_filters(marker_id, landmarker_measurement)
						
						kalman_filter.Apply_EKF(expected_state, landmarker_measurement)
						validity_info = kalman_filter.measurement_validition()
						self.particles[i].update_weight(marker_id)

						if np.linalg.norm(validity_info) < validity_threshold:
							kalman_filter.Update()

					total_weight += self.particles[i].get_weight()

				sum_weights = 0
				for i in range(NUMBER_PARTICLES):
					self.particles[i].normalize_weight(total_weight)
					sum_weights += self.particles[i].get_weight()**2

				self.publisher_pose(self.particles)

				neff = 1.0/sum_weights

				if neff < float(NUMBER_PARTICLES)/2:
					self.particles = resample_particles(self.particles, self.updated_marker)
					del self.updated_marker
					self.updated_marker = [False]*NUMBER_MARKERS

			self.reset_list_ids()

			self.frequency.sleep()

	def publisher_pose(self, particles):

		Max = 0
		Max_id = 0

		for i in range(NUMBER_PARTICLES):
			if particles[i].get_weight() > Max:
				Max = particles[i].get_weight()
				Max_id = i

		estimate = particles[Max_id].get_position()

		pose = Pose()

        # Fill position values
		pose.position.x = estimate[0]
		pose.position.y = estimate[1]
		pose.position.z = 0

        # Convert rpy to quaternion
		quaternion = tf.transformations.quaternion_from_euler(0, 0, estimate[2])

        # Add quaternion to message
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]

		self.posePublisher.publish(pose)

				
	def callback_odom(self, data):

		# robo_frame
		#frame_id = data.header.frame_id # odom
		#child_frame_id = data.child_frame_id # base_link

		# pose
		x = data.pose.pose.position.x # front-back
		y = data.pose.pose.position.y # right-left
				
		orientation_x = data.pose.pose.orientation.x
		orientation_y = data.pose.pose.orientation.y
		orientation_z = data.pose.pose.orientation.z
		orientation_w = data.pose.pose.orientation.w
		_, _, yaw = tf.transformations.euler_from_quaternion((orientation_x, orientation_y, orientation_z, orientation_w))

		if self.first_read == True:
			self.last_position = np.array([x, y, yaw], dtype='float64').transpose()
			self.total_movement = np.array([0, 0, 0], dtype='float64').transpose()
			self.first_read = False

		self.odom_position = np.array([x, y, yaw], dtype='float64').transpose()
		self.movement = np.subtract(self.odom_position, self.last_position)
		self.total_movement = np.add(self.total_movement, np.absolute(self.movement))

		if self.movement[2] > np.pi:
			self.movement[2] = 2*np.pi - self.movement[2]
		if self.movement[2] < -np.pi:
			self.movement[2] = - 2*np.pi - self.movement[2]
		self.last_position = self.odom_position
		self.read_move = np.add(self.read_move, self.movement)

	def actual_movement(self): 
		return self.read_move

	def get_movement(self):
		msg = self.read_move
		self.read_move = np.array([0, 0, 0], dtype='float64').transpose()
		return msg

	def get_total_movement(self):
		return self.total_movement


	def callback_Markers(self, data):
		
		# static tf could be applied here: z = z + z_distance_left_eye_to_robot_wheel, x = x + x_distance_left_eye_to_robot_wheel
		for i in range(NUMBER_MARKERS):
			try:
				marker_info = data.markers.pop()
			except:
				break
		
			self.list_ids[i] = marker_info.id
			self.markers_info[marker_info.id] = marker_info


	def get_measerment(self, index):

		x = self.markers_info[index].pose.pose.position.x # right-left
		z = self.markers_info[index].pose.pose.position.z # front-back

		# position of the marker relative to base_link
		z = z + self.z_distance_left_eye_to_robot_wheel
		x = x + self.x_distance_left_eye_to_robot_wheel

		marker_distance = np.sqrt(z**2+x**2)
		marker_direction = np.arctan(x/z)

		return np.array([marker_distance, -marker_direction], dtype='float64').transpose()


	def get_list_ids(self):
		return self.list_ids

	def reset_list_ids(self):
		i = 0
		while self.list_ids[i] != KEY_NUMBER:
			self.list_ids[i] = KEY_NUMBER
			i += 1

	def marker_info(self, index):
		return self.markers_info[index]

if __name__ == '__main__':

	f = FastSlam()
	f.run()