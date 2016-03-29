#!/usr/bin/python

import cv2
import img_analysis
import os
import rospy
from std_msgs.msg import String

class Camera2D():
    """This class is responsible for interacting with an HD Camera."""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Capture_2D', String, \
                                           self.callback_2d_capture)

        # ROS publishments
        self.done_capture = rospy.Publisher('Done_Capture', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)

        # Camera init
        self.camera_name = 'usb-HD_Camera_Manufacturer_HD_USB_Camera-video-index0'
        self.camera_id = self.get_video_id()
        self.camera = cv2.VideoCapture(self.camera_id)
        #self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1944)
        #self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 2592)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1536)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 2048)

        # Take dummy picture
        self.get_image()
        print('Camera ready for operation.')

    def get_video_id(self):
        """Returns the id (#) of /dev/video# of self.camera_name."""
        camera_connected = True
        v4l_path = '/dev/v4l/by-id/'
        try:
            available_cameras = os.listdir(v4l_path)
            if self.camera_name in available_cameras:
                return int(os.path.realpath(os.path.join(v4l_path, \
                                                self.camera_name))[-1])
        except OSError:
            pass
        raise EnvironmentError('Requested camera is not connected.')

    def get_image(self):
        for i in xrange(15):
            self.camera.grab()
            _, img = self.camera.retrieve()
        return img

    def callback_2d_capture(self, data):
        filename = data.data
        #cwd = os.getcwd()
        cwd = '/home/ubuntu/Desktop/Petri/'
        file_path = os.path.join(cwd, filename)
        # if not self.camera.grab():
        #     print('Error grabbing camera frame.')
        #     return
        # _, img = self.camera.retrieve()

        img = self.get_image()

        # img_dict = {filename: img}
        # img_analysis.imshow(img_dict)
        cv2.imwrite(file_path, img)
        print("Image {0} saved in {1}".format(filename, cwd))
        self.done_capture.publish(file_path)

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        cam = Camera2D()
        cam.listener()

    except (rospy.ROSInterruptException, EnvironmentError) as e:
        print(e)

