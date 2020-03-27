#! /usr/bin/env python

import numpy as np

number_of_dimensions = 2
Sensor_noise = 0.1

class KalmanFilter():

	def __init__(self):

		# X_ espected value of landmarks' position (x,y)
		# X_robot (x, y, yaw)
		# G gradient of markers' relative position to robot (h:(x_m,y_m) -> (distance, orientation); G = dh/dX_ and X_ = X(k+1|k)) 
		# S covariance matrix markers' position
		# Q covariance matrix markers' measurement
		# V diference between measurement and estimated markers' position

		self.first = True
		self.Q_t = np.identity(number_of_dimensions, dtype='float64')*Sensor_noise #sensor noise


	def compute_G(self, X_robot):

		x = 0 # x position
		y = 1 # y position

		y = self.X_[y] - X_robot[y]
		x = self.X_[x] - X_robot[x]

		# compute G
		denominator = x**2 + y**2
		h11 = x / np.sqrt(denominator)
		h12 = y / np.sqrt(denominator)
		h21 = -y / denominator
		h22 = x / denominator
		self.G = np.array([[h11, h12], [h21, h22]])


	def Apply_EKF(self, X_robot, Z):

		x = 0 # x position
		y = 1 # y position
		o = 2 # o orientaio

		d = 0 # distance measured
		fi = 1 # orientaion of the measurement

		if self.first == True:
			# the angle is in the direction y to x, reverse of the usual x to y
			angle = (X_robot[o] + Z[fi])

			self.X_ = np.array([X_robot[x] + Z[d]*np.cos(angle), X_robot[y] + Z[d]*np.sin(angle)], dtype='float64').transpose() # first landmark position

			self.compute_G(X_robot)
			G_inv = np.linalg.inv(self.G)
			self.S = G_inv.dot(self.Q_t).dot(G_inv.T)

			self.V = np.array([0, 0], dtype='float64').transpose()
			self.Q = np.identity(number_of_dimensions, dtype='float64')

		else:
			# Prediction
			y = self.X_[y] - X_robot[y]
			x = self.X_[x] - X_robot[x]

			d = np.sqrt(x**2 + y**2) # distance
			fi = np.arctan2(y, x) - X_robot[o] # direction

			while fi > np.pi:
				fi = fi - 2*np.pi
			while fi < -np.pi:
				fi = fi + 2*np.pi

			Z_ = np.array([d, fi], dtype='float64').transpose()

			self.compute_G(X_robot)
			self.Q = self.G.dot(self.S).dot(self.G.T) + self.Q_t

			# Observation
			self.V = np.subtract(Z, Z_) # Z = [d, teta] 

	def Update(self):
		# Update
		if self.first == False:
			# K kalman gain
			K = self.S.dot(self.G.T).dot(np.linalg.inv(self.Q))
			#K = self.S.dot(self.G).dot(np.linalg.inv(self.Q))

			self.X_ = self.X_ + K.dot(self.V)
			self.S = (np.identity(number_of_dimensions)- K.dot(self.G)).dot(self.S)
			#self.S = (np.identity(number_of_dimensions)- K.dot(self.G.T)).dot(self.S)		
		else:
			self.first = False

	def get_marker_position(self):
		return self.X_

	def get_marker_covariance(self):
		return self.Q

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