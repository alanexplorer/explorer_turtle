#! /usr/bin/env python

__author__ = "Alan Pereira da Silva"
__copyright__ = "Copyright (C), Daedalus Roboti"
__license__ = "GPL"
__version__ = "1.0"
__email__ = "aps@ic.ufal.br"

import rospy
import cv_bridge
import cv2
from cv2 import aruco #pylint:disable=no-name-in-module
from sensor_msgs.msg import Image, CameraInfo
from explorer_turtle.msg import Marker, MarkerArray #pylint:disable=import-error
import numpy as np
from math import sin, cos, sqrt, atan2, pi

'''
Launch file needs to launch usb_cam_node: rosrun usb_cam usb_cam_node .To get around the error, 
first roslaunch usb_cam usb_cam-test.launch. Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image
'''

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
ARUCO_SQUARE_SIZE = 0.177800
MX_MARKER = 1024

# The grid board type we're looking for
board = aruco.GridBoard_create(
        markersX = 1,
        markersY = 1,
        markerLength = ARUCO_SQUARE_SIZE, # In meters
        markerSeparation = 0.1, # In meters
        dictionary = ARUCO_DICT
    )

rvecs, tvecs = None, None

class ArucoDetector():

    def __init__(self):

        rospy.loginfo("publisher aruco")

        # define ros param
        cam_img = rospy.get_param("/camera_image")
        cam_inf = rospy.get_param("/camera_info")

        self.image_sub = rospy.Subscriber(cam_img, Image, callback=self.image_callback)
        self.caminfo_sub = rospy.Subscriber(cam_inf, CameraInfo, callback=self.caminfo_callback)
        self.marker_image_pub = rospy.Publisher('/explorer/marker/image', Image, queue_size=100)
        self.marker_pub = rospy.Publisher('/explorer/marker', Marker, queue_size=1)
        self.marker_array_pub = rospy.Publisher('/explorer/markers', MarkerArray, queue_size=100)

        self.bridge = cv_bridge.CvBridge()
        self.font = cv2.FONT_HERSHEY_PLAIN

        self.CAMERA_MATRIX = None
        self.DISTORTION_COEFFICIENTS = None
        self.have_cam_info = False

        self.marker_list = MarkerArray()
        self.marker_list.markers = [Marker()]*MX_MARKER
        self.marker_list_status = [False]*MX_MARKER

        self.rate = rospy.Rate(10) # 10hz

    def caminfo_callback(self, caminfo_msg):

        if self.have_cam_info:
            pass
        else:
            if caminfo_msg.K == [0, 0, 0, 0, 0, 0, 0, 0, 0]:
                rospy.logwarn("CameraInfo message has K matrix all zeros")
            else:
                self.marker_list.header = caminfo_msg.header
                self.DISTORTION_COEFFICIENTS = caminfo_msg.D
                self.CAMERA_MATRIX = np.zeros((3, 3))

                for i in range(0, 3):
                    for j in range(0, 3):
                        self.CAMERA_MATRIX[i][j] = caminfo_msg.K[i * 3 + j]

                self.have_cam_info = True

    def image_callback(self, img_msg):

        if self.have_cam_info:

            try:
                cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            except cv_bridge.CvBridgeError as e:
                rospy.loginfo(e)

            gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, ARUCO_SQUARE_SIZE, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS)

            test_img = cv_img

            if len(corners) > 0:
                for i in range(ids.size):
                    aruco_detect = Marker()
                    id_number = ids[i, 0]
                    aruco_detect.id = id_number
                    if id_number < MX_MARKER:
                        self.marker_list_status[i] = True
        
                    test_img = aruco.drawDetectedMarkers(test_img, corners, ids)
                    test_img = aruco.drawAxis(test_img, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS, rvecs[i], tvecs[i], ARUCO_SQUARE_SIZE)
                   
                    #zDist = np.asscalar(tvecs[0][0][2])
                    #camera_matrix, _ = cv2.Rodrigues(rvecs[0]) # calculate your object pose rotation matrix
                    #camera_matrix = np.matrix(camera_matrix).T # get the transpose
                    #camera_rotate, _ = cv2.Rodrigues(camera_matrix) # calculate your camera rvec
                    #yaw = camera_rotate[2][0]

                    yaw = -1 * atan2(tvecs[0][0][0], tvecs[0][0][2])
                    zDist = sqrt(tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2 + tvecs[0][0][2] ** 2)

                    cv2.putText(test_img, "%d tag - %.2f cm - %.2f deg" % (id_number, (zDist * 100), yaw / pi * 180), (0, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
                    aruco_detect.distance = zDist
                    aruco_detect.angle = yaw #radians, which is between PI and -PI
                    self.marker_pub.publish(aruco_detect)
                    self.marker_list.markers[id_number] = aruco_detect

            rosified_test_img = self.bridge.cv2_to_imgmsg(test_img, encoding="bgr8")
            self.marker_list.detected = self.marker_list_status
            self.marker_image_pub.publish(rosified_test_img)
            self.marker_array_pub.publish(self.marker_list)

    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('aruco_node')

    try:
        e = ArucoDetector()
        e.run()
    except rospy.ROSInterruptException:
        pass