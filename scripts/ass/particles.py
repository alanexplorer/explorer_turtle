#!/usr/bin/env python

import random
import numpy as np
from math import sqrt, atan2, sin, cos, pi, exp

NOISE_TRANSLATION = 0.5
NOISE_ROTATION = pi

class Particle:

	def __init__(self, M):

		self.ArUco = [(10, 2), (5, 7), (8, 12), (3, 6)] #tupla (x, y)
		self.weight = 1/M
		self.predicted_state_est = np.array([0, 0, 0], dtype='float64').transpose()
		self.covariance = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
		# Define the state transition jacobian. 
		self.state_transition_jacobian = np.array([[1,0,0],[0,1,0],[0,0,1]])
		self.meas_residual = 0
		self.meas_noise_covariance = 0.005
		self.H = np.array([[9999, 0 , 0],[0, 1, 0],[0 , 0, 9999]])

		# PRIOR KNOWLEDGE OF STATE
		self.predicted_state_est = 0
		self.predicted_covariance_est = 0
		self.state_trans_uncertainty_noise = 0
		self.measurement =	0
		self.ave_meas_dist = 0
		self.residual_covariance = 0
		#self.updated_state_estimate = 0
	
	def set_residual(self, r):

		self.meas_residual = r
	
	def get_position(self):

		return self.predicted_state_est

	def set_position(self, odom):

		x = odom[0]*random.random()*NOISE_TRANSLATION
		y = odom[1]*random.random()*NOISE_TRANSLATION
		yaw = odom[2]*random.random()*NOISE_ROTATION

		if yaw > pi:
			yaw = 2*pi - yaw
		if yaw < -pi:
			yaw = - 2*pi - yaw

		self.predicted_state_est = np.array([x, y, yaw], dtype='float64').transpose()

	def predicted(self, state_trans_uncertainty_noise):

		# Calculated the total uncertainty of the predicted state estimate
		self.predicted_covariance_est = self.state_transition_jacobian*self.predicted_covariance_est*np.transpose(self.state_transition_jacobian)+state_trans_uncertainty_noise
	
	def update(self, ave_meas_dist):
		
		# Calculating the measurement we expect to get based on received odometery data. 
		expected_meas = np.cross(np.array([0, 1, 0]), np.array([self.predicted_state_est[0], self.predicted_state_est[1], self.predicted_state_est[2]]))

		#innovation or measurement residual: The difference between our actual measurement and the measurement we expected to get. 
		self.meas_residual = ave_meas_dist - expected_meas

		#Account for the measurement noise by adding error 
		meas_noise_covariance = 0.005

		# Measurement jacobian: 
		H = np.array([[9999, 0 , 0],[0, 1, 0],[0 , 0, 9999]])

		# Innovation (or residual) covariance
		self.residual_covariance = H*self.predicted_covariance_est*np.transpose(H)+meas_noise_covariance

		# Near-optimal Kalman gain: Amalgamating all the uncertainties
		kalman_gain = self.predicted_covariance_est*np.transpose(H)*np.linalg.inv(self.residual_covariance)

		# Updated state estimate
		self.predicted_state_est =  np.array([self.predicted_state_est[0], self.predicted_state_est[1], self.predicted_state_est[2]]) + np.dot(kalman_gain, self.meas_residual)

		# Updated covariance estimate
		self.predicted_covariance_est= (np.identity(3) - np.cross(kalman_gain,H))*self.predicted_covariance_est

	def update_weight(self):

		std = self.residual_covariance
		dev = self.meas_residual

		fact = sqrt(np.linalg.det(2* pi * std))
		expo = - np.dot(dev.T, np.linalg.inv(std)).dot(dev)/2
		self.weight = self.weight / fact * np.exp(expo)
	
	def get_weight(self):

		return self.weight

	def normalize_weight(self, total_weight):
		if total_weight != 0:
			self.weight = self.weight / total_weight

