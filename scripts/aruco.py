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
from ass.utility import *

# Launch file needs to launch usb_cam_node: rosrun usb_cam usb_cam_node
# To get around the error, first roslaunch usb_cam usb_cam-test.launch
# Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image

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
        self.marker_image_pub = rospy.Publisher('/explorer/marker/image', Image, queue_size=10)
        self.marker_pub = rospy.Publisher('/explorer/marker', Marker, queue_size=1)
        self.marker_array_pub = rospy.Publisher('/explorer/markers', MarkerArray, queue_size=10)

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
                    aruco_detect.id = ids[i, 0]
                    self.markerStatusList(ids[i, 0])
                    test_img = aruco.drawDetectedMarkers(test_img, corners, ids)
                    test_img = aruco.drawAxis(test_img, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS, rvecs[i], tvecs[i], ARUCO_SQUARE_SIZE)
                    #R, _ = cv2.Rodrigues(rvecs[i])
                    #angles = self.rotationMatrixToEulerAngles(R)
                    # angles = angles / pi * 180
                    # roll = normangle(angles[2])
                    # yaw = normangle(angles[1])
                    # pitch = normangle(angles[0])
                    xDist = np.asscalar(tvecs[0][0][0])
                    #yDist = np.asscalar(tvecs[0][i][1])
                    zDist = np.asscalar(tvecs[0][0][2])
                    yaw = atan2(xDist, zDist)
                    #cameraPose = -R.T * tvecs[0] 
                    cv2.putText(test_img, "%.1f cm -- %.0f deg" % ((zDist * 100), yaw), (0, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
                    aruco_detect.distance = zDist
                    aruco_detect.angle = yaw #in degree
                    self.marker_pub.publish(aruco_detect)
                    self.marker_list.markers[ids[i, 0]] = aruco_detect

            rosified_test_img = self.bridge.cv2_to_imgmsg(test_img, encoding="bgr8")
            self.marker_list.detected = self.marker_list_status
            self.marker_image_pub.publish(rosified_test_img)
            self.marker_array_pub.publish(self.marker_list)

    def markerStatusList(self, i):

        if i <= MX_MARKER:
            self.marker_list_status[i] = True


    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()

    def rotationMatrixToEulerAngles(self, R):
        # https://www.learnopencv.com/rotation-matrix-to-euler-angles/

        sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = atan2(R[2, 1], R[2, 2])
            y = atan2(-R[2, 0], sy)
            z = atan2(R[1, 0], R[0, 0])
        else:
            x = atan2(-R[1, 2], R[1, 1])
            y = atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

if __name__ == '__main__':
    rospy.init_node('aruco_node')

    try:
        e = ArucoDetector()
        e.run()
    except rospy.ROSInterruptException:
        pass