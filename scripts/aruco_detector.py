#! /usr/bin/env python

import rospy
import cv_bridge
import cv2
from cv2 import aruco #pylint:disable=no-name-in-module
from sensor_msgs.msg import Image, CameraInfo
from explorer_turtle.msg import Marker, MarkerArray #pylint:disable=import-error
from geometry_msgs.msg import Pose
import numpy as np
from math import sin, cos, sqrt, atan2
from tf.transformations import quaternion_from_euler


# Launch file needs to launch usb_cam_node: rosrun usb_cam usb_cam_node
# To get around the error, first roslaunch usb_cam usb_cam-test.launch
# Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

ARUCO_SQUARE_SIZE = 0.097

R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

# The grid board type we're looking for
board = aruco.GridBoard_create(
        markersX = 1,
        markersY = 1,
        markerLength = ARUCO_SQUARE_SIZE, # In meters
        markerSeparation = 0.1, # In meters
        dictionary = ARUCO_DICT
    )

rvecs, tvecs = None, None

class ArucoNode():

    def __init__(self):

        #rospy.loginfo("Starting node")

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.image_callback)
        self.caminfo_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, callback=self.caminfo_callback)
        self.marker_image_pub = rospy.Publisher('/explorer/marker/image', Image, queue_size=10)
        self.marker_pub = rospy.Publisher('/explorer/marker/position', Pose, queue_size=10)
        self.marker_array_pub = rospy.Publisher('/explorer/marker/array', MarkerArray, queue_size=10)

        self.bridge = cv_bridge.CvBridge()

        self.font = cv2.FONT_HERSHEY_PLAIN

        self.CAMERA_MATRIX = None
        self.DISTORTION_COEFFICIENTS = None
        self.have_cam_info = False

        rospy.spin()

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

            markerList = MarkerArray()
            markerList.header = img_msg.header

            try:
                cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            except cv_bridge.CvBridgeError as e:
                rospy.loginfo(e)

            #cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

            gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, ARUCO_SQUARE_SIZE, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS)

            #Tvecs: +x is to the right (as seen by camera), +y is down (as seen by camera), +z is further from camera
            #Rvecs: 

            #rospy.loginfo("ids: {ids}".format(ids=ids))


            test_img = cv_img

            if ids is not None:
                for i in range(len(ids)):
                    test_img = aruco.drawDetectedMarkers(test_img, corners, ids)
                    test_img = aruco.drawAxis(test_img, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS, rvecs[i], tvecs[i], ARUCO_SQUARE_SIZE)
               
            rosified_test_img = self.bridge.cv2_to_imgmsg(test_img, encoding="bgr8")
            self.marker_image_pub.publish(rosified_test_img)

            if ids is not None:
                for i in range(len(ids)):
                    marker = Marker()
                    marker.id = ids[i][0]
                    marker_pose = Pose()

                    marker_pose.position.x = tvecs[i][0][0]
                    marker_pose.position.y = tvecs[i][0][1]
                    marker_pose.position.z = tvecs[i][0][2]

                    #-- Obtain the rotation matrix tag->camera

                    R_ct    = np.matrix(cv2.Rodrigues(rvecs)[0])
                    R_tc    = R_ct.T

                    #roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

                    R=R_flip*R_tc
                    sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
                    singular = sy < 1e-6
                    if not singular:
                        x = atan2(R[2, 1], R[2, 2])
                        y = atan2(-R[2, 0], sy)
                        z = atan2(R[1, 0], R[0, 0])
                    else:
                        x = atan2(-R[1, 2], R[1, 1])
                        y = atan2(-R[2, 0], sy)
                        z=0

                    roll=x
                    pitch=y
                    yaw=z
                    
                    (q0,q1,q2,q3) = quaternion_from_euler(roll,pitch,yaw)

                    '''
                    #Conversion from axis-angle to quaternion
                    #Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/

                    #Get total angle of rotation from axis-angle
                    angle = cv2.norm(rvecs[i][0])
                    #Normalize axis-angle vector
                    axis = (rvecs[i][0][0] / angle, rvecs[i][0][1] / angle, rvecs[i][0][2] / angle)

                    x = axis[0] * sin(angle / 2)
                    y = axis[1] * sin(angle / 2)
                    z = axis[2] * sin(angle / 2)
                    w = cos(angle / 2)
                    '''

                    marker_pose.orientation.x = q0
                    marker_pose.orientation.y = q1
                    marker_pose.orientation.z = q2
                    marker_pose.orientation.w = q3

                    marker.pose.pose = marker_pose

                    self.marker_pub.publish(marker_pose)

                    markerList.markers.append(marker)

            self.marker_array_pub.publish(markerList)


if __name__ == '__main__':
    rospy.init_node('aruco_node')

    try:
        n = ArucoNode()
    except rospy.ROSInterruptException:
        pass