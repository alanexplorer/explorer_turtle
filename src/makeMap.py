#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@authors: 		Alan Pereira da Silva
@description:
				class which handles odometry and laser data
				updated grid map based on Bayesian probabillistics
				and publishes the OccupanyGrid message
@date:    		05.01.2020
@version: 		1.0.0
@requirements:  numpy 1.13.3 -- install command: pip install numpy==1.13.3
"""

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
import numpy as np
from eigen import transform
from Bresenham import *

class makeMap:

    def __init__(self):

        # initialize relevant variables

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.curX = 0.0
        self.curY = 0.0

        self.prior = [] #array with probability prior
        self.laserInit = False
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_size = 0
        self.n_samples = 0
        self.pointsX = []
        self.pointsY = []
        self.curRanges = []

        self.robot_odom = None

        # parameter for mapping
        self.GRID_SIZEX = 500
        self.GRID_SIZEY = 500
        self.GRID_RESOLUTION = 0.05 #in meters/cell(5cm)

        self.LASER_MIN_RANG = 0.5
        self.LASER_MAX_RANG = 8
        self.SCAN_DIFF_THRESHOLD = 0.05

        self.SENSOR_MODEL_TRUE_POSITIVE = 0.9
        self.SENSOR_MODEL_FALSE_POSITIVE = 0.3

        self.OCCUPIED_PROB = 0.5

        self.current_map = OccupancyGrid()

        # define subscribers

        rospy.Subscriber('odom',Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # define publishers
        self.pub_map = rospy.Publisher('/frontier_map',OccupancyGrid, queue_size=100)

        self.initializeMap()

        #publish rate
        self.rate = rospy.Rate(10)

        rospy.loginfo("publishing updated map.")

    def run(self):
        while not rospy.is_shutdown():
            self.updateMap() #do an update map per point  
            self.updateOdom()
            self.rate.sleep()
    
    def odom_callback(self, msg):
        #this subscriber save the current orientation and position
        orientation_q = msg.pose.pose.orientation # take the quaternions
        #convert to euler, we only care about the Z rotation, the yaw
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion (orientation_list)
        #we lets use just the yaw for robot mobilie
        self.curX = msg.pose.pose.position.x
        self.curY = msg.pose.pose.position.y
        #print(self.yaw)
        self.robot_odom = msg

    def scan_callback(self, msg):
        self.curRanges = []
        if(self.laserInit == False):
            self.laserIntialed(msg)

        for i in msg.ranges:
            if(not math.isnan(i)):
                self.curRanges.append(i)

    def laserIntialed(self, msg):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_size = len(msg.ranges)
        self.laserInit = True

    def initializeMap(self):
        #legend:
        # -1 ==> unknowm
        # 0  ==> free
        # 100 => occupied

        #Initialize the map as unknown -1 
        #Initiallize prior as 0.5
        
        for i in range(self.GRID_SIZEX*self.GRID_SIZEY):
            self.current_map.data.append(-1.0)
            self.prior.append(0.5)

        # define messages to be published
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


    def updateOdom(self):
        # first, we'll publish the transform over tf
        br = tf.TransformBroadcaster()
        br.sendTransform((self.robot_odom.pose.pose.position.x, self.robot_odom.pose.pose.position.y, self.robot_odom.pose.pose.position.z),
		(self.robot_odom.pose.pose.orientation.x,
		self.robot_odom.pose.pose.orientation.y,
		self.robot_odom.pose.pose.orientation.z,
		self.robot_odom.pose.pose.orientation.w),
		rospy.Time.now(),
		"odom",
		"map")

    def updateMap(self):

        i = 0
        for r in self.curRanges:
            self.updateMapByOneScan(i, r)
            i = i+1
        self.pub_map.publish(self.current_map)

    
    def updateMapByOneScan(self, i, r):

        x = r * math.cos(self.angle_min + i*self.angle_increment)
        y = r * math.sin(self.angle_min + i*self.angle_increment)

        pointWorld = self.laser2world(x, y)
        
        gridXY = self.point2grid(pointWorld.x, pointWorld.y)

        self.updateCell(gridXY.x, gridXY.y)
            

    def updateCell(self, x, y):

        obstacleCellIndex = int(self.gridXY2mapIndex(x, y))
        self.updateObstacleCell(obstacleCellIndex, x, y)
    
    def updateObstacleCell(self, index, x, y):

        if index < 0 or index > self.GRID_SIZEX*self.GRID_SIZEY:
            print("out of map index")
        else:
            p_z = self.SENSOR_MODEL_TRUE_POSITIVE*self.prior[index] + self.SENSOR_MODEL_FALSE_POSITIVE*(1-self.prior[index])

            self.prior[index] = (self.SENSOR_MODEL_TRUE_POSITIVE*self.prior[index])/p_z # p(m|z)

            if self.prior[index] > 0.5:
                self.current_map.data[index] = 100
                self.updateCellFree(x, y)
            else:
                self.current_map.data[index] = 0

    def updateCellFree(self, x, y):

        pose_robot = self.point2grid(self.curX, self.curY)
        
        empty_cell = bresenham(pose_robot.x, pose_robot.y, x, y)

        for empty_coord in empty_cell[:-1]:
            index = self.gridXY2mapIndex(empty_coord[0], empty_coord[1])
            self.current_map.data[index] = 0

    def laser2world(self, x, y):

        t = transform()
        t.getRotationMatrix(self.yaw)
        t.getTranslationMatrix(self.curX, self.curY)
        p = Point()
        (p.x, p.y) = t.pointTransform(x,y)
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

    def gridXY2mapIndex(self, x, y):

        return y*self.GRID_SIZEX+x

if __name__ == '__main__':
    try:
        rospy.init_node("map_grid")
        m = makeMap()
        m.run()
        
    except rospy.ROSInterruptException:
        pass