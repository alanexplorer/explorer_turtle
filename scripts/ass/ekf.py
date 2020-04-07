#! /usr/bin/env python

import numpy as np
from math import cos, sin, sqrt, atan2

class KalmanFilter():

	def __init__(self):

		# JA - Jacobian of measurement model with respect to landmarker pose
		# P  - Innovation covariance matrix
		# Q  - Linearized vehicle measurement noise
		# V  - Diference between measurement and estimated markers' position

		self.JA = np.array([[0, 0], [0, 0]]) 
		self.P = 0
		self.Q = np.identity(2, dtype='float64')*0.1 #sensor noise
		self.V = np.array([0, 0], dtype='float64').transpose()


	def setJA(self, JA):
		self.JA = JA

	def setQ(self, Q):
		self.Q = Q

	def Apply_EKF(self, robot, Z):

		self.X_ = np.array([robot[0] + Z[0]*cos(Z[1]), robot[1] + Z[0]*sin(Z[1])], dtype='float64').transpose()

		# Prediction
		y = self.X_[1] - robot[1]
		x = self.X_[0] - robot[0]

		d = sqrt(x**2 + y**2) # distance
		fi = atan2(y, x) - X_robot[o] # direction

			while fi > np.pi:
				fi = fi - 2*np.pi
			while fi < -np.pi:
				fi = fi + 2*np.pi

			Z_ = np.array([d, fi], dtype='float64').transpose()

			self.compute_G(X_robot)
			self.Q = self.G.dot(self.S).dot(self.G.T) + self.Q_t

			# Observation
			self.V = np.subtract(Z, Z_) # Z = [d, teta] 


		#Calculating the measurement we expect to get based on received odometery data. 
		expected_meas = np.cross(np.array([0, 1, 0]), np.array([robot[0], robot[1], robot[2]]))
		self.V = Z[0] - expected_meas

		self.P = self.JA*self.P*self.JA.T + self.Q

	def Update(self):

		# K kalman gain
		K = (self.P*self.JA.T) * np.linalg.inv(self.P)	

	def get_marker_position(self):
		return self.X_

	def get_marker_covariance(self):
		return self.P

	def get_marker_validity(self):
		return self.V

	def measurement_validition(self):
		return np.dot(self.V.T, np.linalg.inv(self.Q)).dot(self.V)

	def copy(self):
		new_KF = KalmanFilter()
		new_KF.X_ = np.copy(self.X_)
		new_KF.Q = np.copy(self.Q)
		new_KF.V = np.copy(self.V)
		new_KF.S = np.copy(self.S)
		new_KF.first = False

		return new_KF
