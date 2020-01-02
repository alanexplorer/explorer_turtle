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

class makeMap:

    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    curX = 0.0
    curY = 0.0
    curRanges = []
    prevRanges = []
    prior = []
    laserInit = False
    angle_min = 0.0
    angle_increment = 0.0
    range_size = 0

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
            #self.updateOdom()
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
        
        for i in range(self.range_size):
            self.prevRanges[i] = self.curRanges[i]
            self.curRanges[i] = msg.ranges[i] if msg.ranges[i] < self.LASER_MAX_RANG else float("NaN")

            #print(self.curRanges[i])

    def laserIntialed(self, msg):
        for i in msg.ranges:
            self.curRanges.append(i)
            self.prevRanges.append(i)

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


if __name__ == '__main__':
    try:
        rospy.init_node("map_publisher")
        makeMap()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass