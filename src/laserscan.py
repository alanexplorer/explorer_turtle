#!/usr/bin/env python

import rospy
import random
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import pyplot as plt
from sklearn import linear_model, datasets

class percepition():

    data = [] 
    n_samples = 0
    pointsX = []
    pointsY = []

    def __init__(self):

        sub = rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=100)

        rate = rospy.Rate(10)

        rospy.loginfo("updated")

        while not rospy.is_shutdown():
            if self.n_samples:
                self.ransac()
            rate.sleep()

    def ransac(self):
        X = self.pointsX.reshape(len(self.pointsX), 1)
        Y = self.pointsX.reshape(len(self.pointsY), 1)
        # Fit line using all data
        lr = linear_model.LinearRegression()
        lr.fit(X, Y)

        # Robustly fit linear model with RANSAC algorithm
        ransac = linear_model.RANSACRegressor()
        ransac.fit(X, Y)
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)

        # Predict data of estimated models
        line_X = np.arange(X.min(), X.max())[:, np.newaxis]
        line_y = lr.predict(line_X)
        line_y_ransac = ransac.predict(line_X)

        lw = 2
        plt.scatter(X[inlier_mask], Y[inlier_mask], color='yellowgreen', marker='.', label='Inliers')
        plt.scatter(X[outlier_mask], Y[outlier_mask], color='gold', marker='.', label='Outliers')
        plt.plot(line_X, line_y, color='navy', linewidth=lw, label='Linear regressor')
        plt.plot(line_X, line_y_ransac, color='cornflowerblue', linewidth=lw, label='RANSAC regressor')
        plt.legend(loc='lower right')
        plt.xlabel("Input")
        plt.ylabel("Response")
        plt.show()

    def callback(self, msg):
        #print(len(msg.ranges))
        arrx = []
        arry = []
      
        for i in range(len(msg.ranges)):

            if(not math.isnan(msg.ranges[i])):
                x = msg.ranges[i] * math.cos(msg.angle_min + i*msg.angle_increment)
                y = msg.ranges[i] * math.sin(msg.angle_min + i*msg.angle_increment)
                arrx.append(x)
                arry.append(y)
               
        self.pointsX = np.array(arrx) #Convert a list into an array
        self.pointsY = np.array(arry) #Convert a list into an array
        self.data = np.asarray([self.pointsX, self.pointsY])
        self.n_samples = len(arry)


if __name__ == '__main__':
    try:
        rospy.init_node('scan_values')
        percepition()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass