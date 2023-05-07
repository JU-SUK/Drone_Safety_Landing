#!/usr/bin/env python3

from __future__ import print_function
import time
import roslib
import sys
import rospy
import cv2
import glob
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from kalman_filter import ekfilter

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/cv_image", Image)
        self.distance_pub = rospy.Publisher("/relative_distance", Float32MultiArray, queue_size=1)

        self.bridge = CvBridge()
        # 나중에 /stereo/left/image_raw변경하자!!!
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.lostnumber = 0

    def callback(self, data):
        try:
            # transform ROS image message into opencv image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        global ret, mtx, dist, rvecs, tvecs

        # aruco basic setting
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

        # hese parameters include things like marker detection thresholds, corner refinement methods, and adaptive thresholding parameters
        # we should change these parameters so that can achieve the desired level of marker detection accuracy and robustness
        parameters = aruco.DetectorParameters_create()

        # convert the image
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # detect marker configuration
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image_gray, aruco_dict, parameters=parameters)

        if np.all(ids != None):
            self.lostnumber = 0
            id = ids[0][0]
            lock_number = 0
            for i in range(ids.size):
                if ids[i][0] > id:
                    id = ids[i][0]
                    lock_number = i
            marker_size = 0

            if id==1:
                marker_size = 0.139
            elif id==0:
                marker_size = 0.02
            elif id==2:
                marker_size = 0.071
            elif id==3:
                marker_size = 0.0325
            elif id==4:
                marker_size = 0.016
            # pose estimation
            # 0.19: markerLength, mtx: cameraMatrix, dist: distortion coefficients
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[lock_number], marker_size, mtx, dist)

            # read corners information
            top_left_X = corners[0][0][0][0]
            top_left_Y = corners[0][0][0][1]

            top_right_X = corners[0][0][1][0]
            top_right_Y = corners[0][0][1][1]

            bottom_left_X = corners[0][0][2][0]
            bottom_left_Y = corners[0][0][2][1]

            bottom_right_X = corners[0][0][3][0]
            bottom_right_Y = corners[0][0][3][1]

            # get pose information
            cor_x = tvec[0][0][0]
            cor_y = tvec[0][0][1]
            cor_z = tvec[0][0][2]
            f = ekfilter(cor_x, cor_y)
            cor_x = f[0][0]
            cor_y = f[1][0]

            print("x=", cor_x)
            print("y=", cor_y)
            print("z=", cor_z)

            midpoint_X = (top_left_X + bottom_right_X) / 2
            midpoint_Y = (top_left_Y + bottom_right_Y) / 2

            # draw axis and detect marker
            #aruco.drawAxis(cv_image, mtx, dist, rvec[0], tvec[0], 0.01)
            aruco.drawDetectedMarkers(cv_image, corners)

            # draw ID text on top of image
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, "X: {}".format(cor_x), (0,364), font, 1, (0,255,0), 2, cv2.LINE_AA)
            cv2.putText(cv_image, "Y: {}".format(cor_y), (0, 400), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(cv_image, "Z: {}".format(cor_z), (0, 436), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # incorporate pose information together and print on image
            dis = Float32MultiArray()
            dis.data = (cor_x, cor_y, cor_z)

            cv2.imshow("Image", cv_image)
            cv2.waitKey(3)

            # Node publish - pose information
            self.distance_pub.publish(dis)

            # Node publish - cv_image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
        else:
            self.lostnumber += 1
            if self.lostnumber > 100:
                dis = Float32MultiArray()
                dis.data = (float('nan'), float('nan'), float('nan'))
                self.distance_pub.publish(dis)


def main(args):
    global ret, mtx, dist, rvecs, tvecs

    # cameraMatrix and distortion coefficents
    mtx = np.array([[552.1877667, 0. , 289.37921553], [ 0. , 550.98791255, 228.87373308], [0. , 0. , 1.]])
    dist = np.array([[0, 0, 0, 0, 0]])

    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
