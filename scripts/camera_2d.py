#!/usr/bin/python

import cv2
import datetime
import os
import rospy
import sys

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Camera2D():
    """This class is responsible for interacting with an HD Camera."""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('image_raw', Image, \
                                           self.callback_2d_capture)

        self.bridge = CvBridge()
        self.flag = False

    def callback_2d_capture(self, data):
        self.flag = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        filename = "{0}.jpg".format(str(datetime.datetime.now())[:19])
        cwd = '/home/marc/Desktop/photos'
        file_path = os.path.join(cwd, filename)

        cv2.imwrite(file_path, cv_image)
        print("Image {0} saved in {1}".format(filename, cwd))

    def listener(self):
        while not self.flag:
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        cam = Camera2D()
        cam.listener()

    except (rospy.ROSInterruptException, EnvironmentError) as e:
        print(e)

