#! /usr/bin/env python

import rospy
import cv_bridge
import cv2
from cv2 import aruco #pylint:disable=no-name-in-module
from sensor_msgs.msg import Image, CameraInfo
#from explorer_turtle.msg import Marker, MarkerArray #pylint:disable=import-error
#from geometry_msgs.msg import Pose
import numpy as np
from math import sin, cos, sqrt, atan2, pi

# Launch file needs to launch usb_cam_node: rosrun usb_cam usb_cam_node
# To get around the error, first roslaunch usb_cam usb_cam-test.launch
# Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
ARUCO_SQUARE_SIZE = 0.1 # 100 cm
MX_MARKER = 20

# The grid board type we're looking for
board = aruco.GridBoard_create(
        markersX = 1,
        markersY = 1,
        markerLength = ARUCO_SQUARE_SIZE, # In meters
        markerSeparation = 0.1, # In meters
        dictionary = ARUCO_DICT
    )

rvecs, tvecs = None, None

class Aruco():

    def __init__(self):

        rospy.loginfo("publisher aruco")
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback=self.image_callback)
        #self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.image_callback)
        self.caminfo_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, callback=self.caminfo_callback)
        #self.caminfo_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, callback=self.caminfo_callback)
        self.marker_image_pub = rospy.Publisher('/explorer/marker/image', Image, queue_size=10)
        #self.marker_pub = rospy.Publisher('/explorer/marker/position', Pose, queue_size=10)
        #self.marker_array_pub = rospy.Publisher('/explorer/marker/array', MarkerArray, queue_size=10)

        self.bridge = cv_bridge.CvBridge()

        self.font = cv2.FONT_HERSHEY_PLAIN

        self.CAMERA_MATRIX = None
        self.DISTORTION_COEFFICIENTS = None
        self.have_cam_info = False

        self.markerList = [False]*MX_MARKER

        self.rate = rospy.Rate(10) # 10hz

    def caminfo_callback(self, caminfo_msg):

        if self.have_cam_info:
            pass
        else:
            if caminfo_msg.K == [0, 0, 0, 0, 0, 0, 0, 0, 0]:
                rospy.logwarn("CameraInfo message has K matrix all zeros")
            else:
                self.DISTORTION_COEFFICIENTS = caminfo_msg.D
                self.CAMERA_MATRIX = np.zeros((3, 3))

                for i in range(0, 3):
                    for j in range(0, 3):
                        self.CAMERA_MATRIX[i][j] = caminfo_msg.K[i * 3 + j]

                self.have_cam_info = True

    def image_callback(self, img_msg):

        if self.have_cam_info:

            #markerList = MarkerArray()
            #markerList.header = img_msg.header

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
                    self.markerStatusList(ids[i, 0])
                    test_img = aruco.drawDetectedMarkers(test_img, corners, ids)
                    test_img = aruco.drawAxis(test_img, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS, rvecs[i], tvecs[i], ARUCO_SQUARE_SIZE)
                    cv2.putText(test_img, "%.1f cm -- %.0f deg" % ((tvecs[0][0][2] * 100), (rvecs[0][0][2] / pi * 180)), (0, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
                    R, _ = cv2.Rodrigues(rvecs[0])
                    cameraPose = -R.T * tvecs[0]            

            rosified_test_img = self.bridge.cv2_to_imgmsg(test_img, encoding="bgr8")

            self.marker_image_pub.publish(rosified_test_img)

    def markerStatusList(self, i):

        if i <= MX_MARKER:
            self.markerList[i] = True


    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('aruco_node')

    try:
        e = Aruco()
        e.run()
    except rospy.ROSInterruptException:
        pass