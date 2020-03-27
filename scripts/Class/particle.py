#! /usr/bin/env python

import numpy as np
from kalman_filter import KalmanFilter


NUMBER_PARTICLES = 100
NUMBER_MARKERS = 4
translation_noise = 0.1
rotation_noise = 0.1
noise_factor = 1

class Particle():

	#each particle has a pose(x,y,o), a weight(w) and a series of kalman filters for every landmark
	#in the beggining all particles are in the origin frame of the world (0,0,0)

	def __init__(self):

		self.X_robot = np.array([0, 0, 0], dtype='float64').transpose()
		self.weight = 1.0/NUMBER_PARTICLES
		self.Landmarkers = [None]*NUMBER_MARKERS
		self.x_path = np.array([0], dtype='float64')
		self.y_path = np.array([0], dtype='float64')


	def get_kalman_filters(self, marker_id, Z):

		if self.Landmarkers[marker_id] == None:
			self.Landmarkers[marker_id] = KalmanFilter()

		return self.Landmarkers[marker_id]


	def particle_prediction(self, motion_model):

		#if the robot moves we just add the motion model to the previous pose to predict the particle position
		x = 0
		y = 1
		o = 2

		noise = np.array([np.random.normal(0,translation_noise), np.random.normal(0,translation_noise), np.random.normal(0,rotation_noise)], dtype='float64').transpose()
		noise = noise*motion_model*noise_factor

		self.X_robot = self.X_robot + motion_model + noise

		while self.X_robot[o] > np.pi:
			self.X_robot[o] = self.X_robot[o] - 2*np.pi
		while self.X_robot[o] < -np.pi:
			self.X_robot[o] = self.X_robot[o] + 2*np.pi

		self.x_path = np.insert(self.x_path, 0, self.X_robot[x])
		self.y_path = np.insert(self.y_path, 0, self.X_robot[y])

		return self.X_robot


	def update_weight(self, marker_id):

		std = self.Landmarkers[marker_id].get_marker_covariance()
		dev = self.Landmarkers[marker_id].get_marker_validity()

		fact = np.sqrt(np.linalg.det(2* np.pi * std))
		expo = - np.dot(dev.T, np.linalg.inv(std)).dot(dev)/2
		self.weight = self.weight / fact * np.exp(expo)

	def get_weight(self):
		return self.weight

	def normalize_weight(self, total_weight):
		self.weight = self.weight / total_weight

	def set_weight(self, new_weight):
		self.weight = new_weight

	def get_position(self):
		return self.X_robot

	def get_landmarkers(self):
		return self.Landmarkers

	def get_path(self):
		return self.x_path, self.y_path

	def copy(self, updated_marker):
		new_particle = Particle()

		del new_particle.x_path
		del new_particle.y_path
		del new_particle.Landmarkers
		del new_particle.X_robot 

		new_particle.x_path = np.copy(self.x_path)
		new_particle.y_path = np.copy(self.y_path)
		
		for i in range(len(self.Landmarkers)):
			if self.Landmarkers[i] != None and updated_marker[i] == True:
				self.Landmarkers[i] = self.Landmarkers[i].copy()

		new_particle.Landmarkers = self.Landmarkers
		new_particle.X_robot = np.copy(self.X_robot)

		return new_particle