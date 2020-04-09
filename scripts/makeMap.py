#!/usr/bin/env python

import rospy
import tf
from math import sin, cos, pi,tan, atan2, log
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header
from explorer_turtle.msg import *
from visualization_msgs.msg import Marker
import numpy as np
from ass.bresenham import *
from ass.transformations import point2grid, scan2world
from visualization_msgs.msg import MarkerArray, Marker

class makeMap:

    def __init__(self):
        
        self.laserInit = False
        self.odomInit = False
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_size = 0
        self.curRanges = []
        self.prevRanges = []
        self.LASER_MIN_RANG = 0.5
        self.LASER_MAX_RANG = 10
        self.SCAN_DIFF_THRESHOLD = 0.05

        # parameter for mapping
        self.GRID_SIZEX = 500
        self.GRID_SIZEY = 500
        self.GRID_RESOLUTION = 0.05 #in meters/cell(5cm)
        self.origin_x = -self.GRID_SIZEX/2*self.GRID_RESOLUTION
        self.origin_y = -self.GRID_SIZEY/2*self.GRID_RESOLUTION

        #parameter probabilistic

        self.SENSOR_MODEL_TRUE_POSITIVE = 0.9
        self.SENSOR_MODEL_FALSE_POSITIVE = 0.3
        self.OCCUPIED_PROB = 0.5
        self.prior = [] #array with probability prior
        self.p_free=log(0.3/0.7)
        self.p_occ=log(0.9/0.1)
        self.max_logodd=100.0
        self.max_logodd_belief=10.0

        # define ros param
        laser = rospy.get_param("/laser_topic")
        odom = rospy.get_param("/odometry_topic")

        # define subscribers
        rospy.Subscriber(laser, LaserScan, self.scan_callback)
        rospy.Subscriber(odom,Odometry, self.odom_callback)
        #rospy.Subscriber('state_estimate', Config, self.get_state_estimate)

        # define publishers
        self.pub_map = rospy.Publisher('/frontier_map',OccupancyGrid, queue_size=100)
        self.current_map = OccupancyGrid()
        self.initializeMap()       

        #publish rate
        self.rate = rospy.Rate(50) # 50hz   

        #define transform
        self.listener = tf.TransformListener()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.curX = 0
        self.curY = 0
        self.br = tf.TransformBroadcaster()

        rospy.loginfo("publishing updated map.")

    def run(self):
        while not rospy.is_shutdown():
            self.updateOdom()
            self.updateMap()
            self.rate.sleep()

    def updateMap(self):

        for i in range(self.range_size):
            if not math.isnan(self.curRanges[i]):
                if not math.isnan(self.prevRanges[i]) and abs(self.curRanges[i] - self.prevRanges[i]) > self.SCAN_DIFF_THRESHOLD:
                    r = self.curRanges[i]
                    x = r * cos(self.angle_min + i*self.angle_increment)
                    y = r * sin(self.angle_min + i*self.angle_increment)
                    self.updateMapByOneScan(x,y)

        self.pub_map.publish(self.current_map)

    def updateMapByOneScan(self, x, y):

        pointWorld = scan2world(x, y, self.yaw, self.curX, self.curY)
        originPosition = Point()
        originPosition = self.current_map.info.origin.position
        gridXY =  point2grid(pointWorld[0], pointWorld[1], originPosition.x, originPosition.y, self.GRID_SIZEX, self.GRID_SIZEY, self.GRID_RESOLUTION)   
        self.updateCell(gridXY[0], gridXY[1])
    
    def odom_callback(self, msg):
        
        #this subscriber save the current orientation and position
        orientation_q = msg.pose.pose.orientation # take the quaternions
        #convert to euler, we only care about the Z rotation, the yaw
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion (orientation_list)
        #we lets use just the yaw for robot mobilie
        
        self.curX = msg.pose.pose.position.x
        self.curY = msg.pose.pose.position.y
        self.odomInit = True

    def scan_callback(self, msg):
        if(self.laserInit == False):
            self.laserIntialed(msg)

        for i in range(len(msg.ranges)):
            self.prevRanges[i] = self.curRanges[i]
            self.curRanges[i] = math.nan if msg.ranges[i] > self.LASER_MAX_RANG else msg.ranges[i]

    def laserIntialed(self, msg):
        for i in msg.ranges:
            self.curRanges.append(i)
            self.prevRanges.append(i)

        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_size = len(msg.ranges)
        self.laserInit = True

    def get_state_estimate(self, msg):

        self.curX = msg.x
        self.curY = msg.y
        self.yaw = msg.th

    def initializeMap(self):
        # initialize int map grid array (100 = occupy, 0 = empty, -1 = unknown state)

        for i in range(self.GRID_SIZEX*self.GRID_SIZEY):
            self.current_map.data.append(-1.0)
            self.prior.append(0.0)

        # define messages to be published
        self.current_map.header.frame_id = "map"
        self.current_map.info.resolution = self.GRID_RESOLUTION
        self.current_map.info.width = self.GRID_SIZEY
        self.current_map.info.height = self.GRID_SIZEX
        self.current_map.info.origin.position.x = -self.GRID_SIZEX / 2 * self.GRID_RESOLUTION
        self.current_map.info.origin.position.y = -self.GRID_SIZEX / 2 * self.GRID_RESOLUTION
        self.current_map.info.origin.position.z = 0.0
        self.current_map.info.origin.orientation.x = 0.0
        self.current_map.info.origin.orientation.y = 0.0
        self.current_map.info.origin.orientation.z = 0.0
        self.current_map.info.origin.orientation.w = 1.0

    def updateOdom(self):
        # first, we'll publish the transform over tf

        self.br.sendTransform((self.curX, self.curY, 0),
                                tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                                rospy.Time.now(), "odom", "map")

         
    def updateCell(self, x, y):

        obstacleCellIndex = int(self.gridXY2mapIndex(x, y))

        if obstacleCellIndex >= 0:
            self.updateObstacleCell(obstacleCellIndex, x, y)

        pos_robot = point2grid(self.curX, self.curY, self.origin_x, self.origin_y, self.GRID_SIZEX, self.GRID_SIZEY, self.GRID_RESOLUTION)
        
        empty_cell = bresenham(pos_robot[0], pos_robot[1], x, y)

        for empty_coord in empty_cell[:-1]:
            freeCellIndex = self.gridXY2mapIndex(empty_coord[0], empty_coord[1])
            self.updateCellFree(freeCellIndex)
            #self.current_map.data[index] = 0
    
    def updateObstacleCell(self, index, x, y):

        if index < 0 or index > (self.GRID_SIZEX*self.GRID_SIZEY):
            print("out of map index")

        elif(len(self.prior)==(self.GRID_SIZEX*self.GRID_SIZEY)):

            self.prior[index] += self.p_occ

            if self.prior[index] > self.max_logodd:
                self.prior[index] = 100

            if self.prior[index] >= self.max_logodd_belief:
                self.current_map.data[index] = 100
            else:
                self.current_map.data[index] = 0

    def updateCellFree(self, index):
        if (index < 0 or index > self.GRID_SIZEX * self.GRID_SIZEY):
            print("out of map index")
        else:
            self.prior[index] +=self.p_free

            if self.prior[index] < -self.max_logodd:
                self.prior[index] = -100
            if self.prior[index] >= self.max_logodd_belief:
                self.current_map.data[index] = 100
            else:
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