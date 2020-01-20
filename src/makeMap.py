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
import numpy as np
from bresenham import *
from transformations import point2grid, pointTransform

class makeMap:

    def __init__(self):

        # initialize relevant variables

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.curX = 0
        self.curY = 0

        
        self.laserInit = False
        self.odomInit = False
        self.angle_min = 0.0
        self.range_max = 10
        self.angle_increment = 0.0
        self.range_size = 0
        self.curRanges = []

        # parameter for mapping
        self.GRID_SIZEX = 500
        self.GRID_SIZEY = 500
        self.GRID_RESOLUTION = 0.1 #in meters/cell(10cm)
        self.origin_x = -self.GRID_SIZEX/2*self.GRID_RESOLUTION
        self.origin_y = -self.GRID_SIZEY/2*self.GRID_RESOLUTION

        #parameter probabilistic

        self.SENSOR_MODEL_TRUE_POSITIVE = 0.9
        self.SENSOR_MODEL_FALSE_POSITIVE = 0.3
        self.OCCUPIED_PROB = 0.5
        self.prior = [] #array with probability prior
        self.robot_odom = None

        # define ros param
        laser = rospy.get_param("/laser_topic")
        odom = rospy.get_param("/odometry_topic")

        # define subscribers

        rospy.Subscriber(laser, LaserScan, self.scan_callback)
        rospy.Subscriber(odom,Odometry, self.odom_callback)

        # define publishers
        self.pub_map = rospy.Publisher('/frontier_map',OccupancyGrid, queue_size=10)
        self.current_map = OccupancyGrid()
        self.initializeMap()       

        #publish rate
        self.rate = rospy.Rate(10)

        rospy.loginfo("publishing updated map.")

    def run(self):
        while not rospy.is_shutdown():
            #self.updateMap() #do an update map per point 
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
        self.odomInit = True
        self.robot_odom = msg

    def scan_callback(self, msg):
        self.curRanges = []
        if(self.laserInit == False):
            self.laserIntialed(msg)

        if (self.curX and self.curY):
            previous_coordinates = (-1,-1)
            #angle = self.angle_min
            #for r in msg.ranges:
            for i, r in enumerate(msg.ranges):
                #theta = angle + self.yaw
                #angle += self.angle_increment
                alpha = self.angle_min + i*self.angle_increment
                if(not math.isnan(r)):
                    #x2 = r * math.cos(theta) + self.curX
                    #y2 = r * math.sin(theta) + self.curY
                    x = r*math.cos(alpha)
                    y = r*math.sin(alpha)

                    pointWorld = pointTransform(x, y, self.yaw, self.curX, self.curY)

                    current_coordinates =  point2grid(self.origin_x, self.origin_y, pointWorld[0], pointWorld[1], self.GRID_SIZEX, self.GRID_SIZEY, self.GRID_RESOLUTION)

                    if previous_coordinates != current_coordinates:
                        self.updateCell(current_coordinates[0], current_coordinates[1])
                    previous_coordinates = current_coordinates
        if(self.odomInit):
            self.updateOdom()
            self.pub_map.publish(self.current_map)

    def laserIntialed(self, msg):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_size = len(msg.ranges)
        self.laserInit = True

    def initializeMap(self):
        # initialize int map grid array (100 = occupy, 0 = empty, -1 = unknown state)

        for i in range(self.GRID_SIZEX*self.GRID_SIZEY):
            self.current_map.data.append(-1.0)
            # initialize float prior grid array
            self.prior.append(0.5)

        # define messages to be published
        #self.current_map.header.frame_id = "map"
        self.current_map.info.resolution = self.GRID_RESOLUTION
        self.current_map.info.width = self.GRID_SIZEX
        self.current_map.info.height = self.GRID_SIZEY
        self.current_map.info.origin.position.x = self.origin_x
        self.current_map.info.origin.position.y = self.origin_y
        self.current_map.info.origin.position.z = 0.0
        self.current_map.info.origin.orientation.x = 0.0
        self.current_map.info.origin.orientation.y = 0.0
        self.current_map.info.origin.orientation.z = 0.0
        self.current_map.info.origin.orientation.w = 1.0
        #self.current_map.header.seq = 0
        #self.current_map.header.stamp = rospy.Time.now()
        #self.current_map.info.map_load_time = rospy.Time.now()
        #self.current_map.info.origin = Pose(Point(-self.GRID_SIZEX/2*self.GRID_RESOLUTION,
        #-self.GRID_SIZEY/2*self.GRID_RESOLUTION, 0), Quaternion(0, 0, 0, 1))


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
            
    def updateCell(self, x, y):

        obstacleCellIndex = int(self.gridXY2mapIndex(x, y))
        #print(obstacleCellIndex)
        self.updateObstacleCell(obstacleCellIndex, x, y)
        #print(len(self.prior))
    
    def updateObstacleCell(self, index, x, y):

        if index < 0 or index > (self.GRID_SIZEX*self.GRID_SIZEY):
            print("out of map index")
        if(len(self.prior)==(self.GRID_SIZEX*self.GRID_SIZEY)):
            p_z = self.SENSOR_MODEL_TRUE_POSITIVE*self.prior[index] + self.SENSOR_MODEL_FALSE_POSITIVE*(1-self.prior[index])

            self.prior[index] = (self.SENSOR_MODEL_TRUE_POSITIVE*self.prior[index])/p_z # p(m|z)

            if self.prior[index] > 0.5:
                self.current_map.data[index] = 100
            else:
                self.current_map.data[index] = 0
            
            self.updateCellFree(x, y)

    def updateCellFree(self, x, y):

        pos_robot = point2grid(self.origin_x, self.origin_y, self.curX, self.curY,
             self.GRID_SIZEX, self.GRID_SIZEY, self.GRID_RESOLUTION)
        
        empty_cell = bresenham(pos_robot[0], pos_robot[1], x, y)

        for empty_coord in empty_cell[:-1]:
            index = self.gridXY2mapIndex(empty_coord[0], empty_coord[1])
            self.current_map.data[index] = 0

    def gridXY2mapIndex(self, x, y):

        return y*(self.GRID_SIZEX)+x

if __name__ == '__main__':
    try:
        rospy.init_node("makeMap")
        m = makeMap()
        m.run()
        
    except rospy.ROSInterruptException:
        pass