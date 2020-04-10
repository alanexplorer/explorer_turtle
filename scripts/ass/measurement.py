#!/usr/bin/env python

import rospy
import numpy as np
from math import pi, sqrt, atan2, isnan, sin, cos
from aruco_msgs.msg import MarkerArray
from scipy.spatial import distance

NUMBER_MARKERS = 8
KEY_NUMBER = 1024

class EkfLocalization:

    def __init__(self):

        self.markers_info = [None]*NUMBER_MARKERS
        self.list_ids = np.ones(NUMBER_MARKERS, dtype='int32')*KEY_NUMBER

        rospy.Subscriber('aruco_marker_publisher/markers', MarkerArray, self.marker_callback)

        self.rate = rospy.Rate(10) # 100hz

    def run(self):
        while not rospy.is_shutdown():

            landmarkers_ids = self.list_ids
            for marker_id in landmarkers_ids:
                if marker_id == KEY_NUMBER:
                    break

                '''
                x = self.markers_info[marker_id].pose.pose.position.x
                y = self.markers_info[marker_id].pose.pose.position.y
                z = self.markers_info[marker_id].pose.pose.position.z
                d1 = sqrt(x**2 + y**2)
                d2 = sqrt(x**2 + z**2)
                d3 = sqrt(y**2 + z**2)
                print(d1, d2, d3)
                '''

            self.rate.sleep()
	
    def marker_callback(self, msg):
        
        for i in range(NUMBER_MARKERS):
            try:
                marker_info = msg.markers.pop()
                marker_info.id
            except:
                break
		
            #self.list_ids[i] = marker_info.id
            #self.markers_info[marker_info.id] = marker_info
 
if __name__ == '__main__':
    try:
        rospy.init_node("ekf_localization")
        e = EkfLocalization()
        e.run()
        
    except rospy.ROSInterruptException:
        pass