#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import pyplot as plt
from sklearn import linear_model, datasets
from eigen import transform

class makeMap:

    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    curX = 0.0
    curY = 0.0

    prior = [] #array with probability prior
    laserInit = False
    angle_min = 0.0
    angle_increment = 0.0
    range_size = 0
    n_samples = 0
    pointsX = []
    pointsY = []


    # parameter for mapping
    GRID_SIZEX = 500
    GRID_SIZEY = 500
    GRID_RESOLUTION = 0.05 #in meters/cell(5cm)

    LASER_MIN_RANG = 0.5
    LASER_MAX_RANG = 8
    SCAN_DIFF_THRESHOLD = 0.05

    SENSOR_MODEL_TRUE_POSITIVE = 0.9
    SENSOR_MODEL_FALSE_POSITIVE = 0.3

    OCCUPIED_PROB = 0.5

    current_map = OccupancyGrid()

    def __init__(self):

        pub_map = rospy.Publisher('/frontier_map',OccupancyGrid, queue_size=100) #I create my publisher

        sub_odom = rospy.Subscriber('odom',Odometry, self.odom_callback, queue_size=100)

        sub_laser = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=100)

        self.initializeMap()

        rate = rospy.Rate(10)

        rospy.loginfo("publishing updated map.")

        while not rospy.is_shutdown():
            if self.n_samples:
                self.updateMap()
            self.updateOdom()
            pub_map.publish(self.current_map)
            rate.sleep()
    
    def odom_callback(self, msg):

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion (orientation_list)

        self.curX = msg.pose.pose.position.x
        self.curY = msg.pose.pose.position.y

        #print(self.yaw)

    def scan_callback(self, msg):
        if(self.laserInit == False):
            self.laserIntialed(msg)

        arrx = []
        arry = []
      
        for i in range(len(msg.ranges)):
            if(not math.isnan(msg.ranges[i])):
                x = msg.ranges[i] * math.cos(msg.angle_min + i*msg.angle_increment)
                y = msg.ranges[i] * math.sin(msg.angle_min + i*msg.angle_increment)
                arrx.append(x)
                arry.append(y)
               
        self.pointsX = np.array(arrx) #Convert a list into an array
        self.pointsY = np.array(arry) 
        self.n_samples = len(arry)        

    def laserIntialed(self, msg):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_size = len(msg.ranges)
        self.laserInit = True

    def initializeMap(self):
        #Initialize the map as unknown -1 
        #Initiallize prior as 0.5
        
        for i in range(self.GRID_SIZEX*self.GRID_SIZEY):
            self.current_map.data.append(-1.0)
            self.prior.append(0.5)

        self.current_map.header.frame_id = "map"
        self.current_map.header.seq = 0
        self.current_map.header.stamp = rospy.Time.now()
        self.current_map.info.height = self.GRID_SIZEX
        self.current_map.info.width = self.GRID_SIZEY
        self.current_map.info.resolution = self.GRID_RESOLUTION
        self.current_map.info.map_load_time = rospy.Time.now()
        #self.current_map.info.origin.position.x = -self.GRID_SIZEX/2*self.GRID_RESOLUTION
        #self.current_map.info.origin.position.y = -self.GRID_SIZEY/2*self.GRID_RESOLUTION
        #self.current_map.info.origin.orientation.w = 1
        self.current_map.info.origin = Pose(Point(-self.GRID_SIZEX/2*self.GRID_RESOLUTION,
        -self.GRID_SIZEY/2*self.GRID_RESOLUTION, 0), Quaternion(0, 0, 0, 1))

        #print(self.current_map.info.origin.position.x, self.current_map.info.origin.position.y)

    def updateOdom(self):
        # first, we'll publish the transform over tf
        odom_broadcaster = tf.TransformBroadcaster()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        current_time = rospy.Time.now()
        odom_broadcaster.sendTransform((self.curX, self.curY, 0.), odom_quat, current_time, "map", "odom")

    def updateMap(self):
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

        #plt.scatter(X[inlier_mask], Y[inlier_mask], color='yellowgreen', marker='.', label='Inliers')
        #plt.scatter(X[outlier_mask], Y[outlier_mask], color='gold', marker='.', label='Outliers')
        #plt.plot(line_X, line_y, color='navy', linewidth=2, label='Linear regressor')
        #plt.plot(line_X, line_y_ransac, color='cornflowerblue', linewidth=2, label='RANSAC regressor')
        #plt.legend(loc='lower right')
        #plt.xlabel("Input")
        #plt.ylabel("Response")
        #plt.show()
        start = Point()
        end = Point()

        start.x = line_X[0]
        start.y = line_y_ransac[0]
        end.x = line_X[len(line_X)-1]
        end.y = line_y_ransac[len(line_y_ransac)-1]

        pointStartWorld = Point()
        pointEndtWorld = Point()

        pointStartWorld = self.laser2world(start.x, start.y)
        pointEndtWorld = self.laser2world(end.x, end.y)

        gridStartXY = self.point2grid(pointStartWorld.x, pointStartWorld.y)
        gridEndXY = self.point2grid(pointEndtWorld.x, pointEndtWorld.y)
        


    def laser2world(self, x, y):

        t = transform()
        t.getRotationMatrix(self.yaw)
        t.getTranslationMatrix(self.curX, self.curY)
        p = Point()
        p = t.pointTransform(x,y)
        return p

    def point2grid(self, x, y):

        originPosition = self.current_map.info.origin.position

        gridX = int(round(x - originPosition.x)/self.GRID_RESOLUTION)
        gridY = int(round(y - originPosition.y)/self.GRID_RESOLUTION)
        assert gridX < self.GRID_SIZEX
        assert gridY < self.GRID_SIZEY
        p = Point()
        p.x = gridX
        p.y = gridY

        return p


if __name__ == '__main__':
    try:
        rospy.init_node("map_publisher")
        makeMap()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass