#!/usr/bin/python

# Imports
import datetime
import cv2
from gridfs import GridFS
import os
import numpy as np
import pprint
import pymongo
import rospy
import sys
import time

from cv_bridge import CvBridge, CvBridgeError
from biobot_ros_msgs.msg import BCAMsg
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from bca_cv import BC_finder

class BCA():
    """This class is responsible for interacting with an HD Camera."""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('image_raw', Image, self.callback_2d_capture)
        self.subscriber = rospy.Subscriber('BC_Analysis',BCAMsg, self.callback_bca)
        self.subscriber = rospy.Subscriber('Take_Image', Bool, self.callback_take_image)

        # ROS publisher
        self.bca_done = rospy.Publisher('Done_Module', String, queue_size=10)

        # Variables initialization
        self.bridge = CvBridge()
        self.cv_image = np.zeros((1944,2592,3), np.uint8)

        # Database Client
        self.client = pymongo.MongoClient()

    def callback_take_image(self,data):
        cv2.imwrite("image_raw.jpg",self.cv_image)
        print('Image taken ...')
        return

    def callback_2d_capture(self, data):
        """
        Callback method for 2d capture, it subscribed to UVC_Camera raw_image topics
        It refresh the image saved in self.cv_image at every callback
        """

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

    def callback_bca(self,data):
        """
        Callback method for BCA analysis, it reads BCA_Analysis ROS topics which is
        a BCAMsg.msg topic containing bacterial colonies selection specification.
        It called colony_selection and saved raw image, analysis image, picking
        image and bacterial colonies found parameters to DB.
        """

        time.sleep(30)

        # Colony selection parameters
        self.perimeter_min = data.perimeter_min
        self.perimeter_max = data.perimeter_max
        self.excentricity_min = data.excentricity_min
        self.excentricity_max = data.excentricity_max
        self.area_min = data.area_min
        self.area_max = data.area_max
        self.number_of_colony = data.number_of_colony
        self.color = data.color
        self.picking = data.picking
        self.protocol = data.protocol
        self.step = data.step
        self.pick_number = data.pick_number

        self.color = tuple(ord(c) for c in self.color[1:].decode('hex'))

        print('perimeter min : ' , self.perimeter_min)
        print('perimeter max : ', self.perimeter_max)
        print('excentricity min : ' , self.excentricity_min)
        print('excentricity max : ' , self.excentricity_max)
        print('area min : ' , self.area_min)
        print('area max : ' , self.area_max)
        print('number of colony : ' , self.number_of_colony)
        print ('color : ' , self.color)
        print('picking : ' , self.picking)
        print('protocol : ' , self.protocol)
        print('step : ' , self.step)
        print('pick number : ' , self.pick_number)

        # Sets operation value in function of self.picking
        if self.picking:
            operation = "picking_{}".format(self.pick_number)

        else:
            operation = "analysis"

        # Saves raw image to .jpg and to DB
        cv2.imwrite("raw_image.jpg", self.cv_image)
        self.writeImageDB("raw", self.protocol, self.step, "raw_image.jpg")

        # Calls colony_selection methods
        parameters, image = self.colony_selection()

        # Saves modified image and found parameters to DB
        cv2.imwrite("temp_image.jpg", image)
        self.writeParamsDB(operation, self.protocol, self.step, parameters)
        self.writeImageDB(operation, self.protocol, self.step, "temp_image.jpg")

        print('BC_Analysis done ...')

        # Publish done
        self.bca_done.publish("BC_Analysis")


    def colony_selection(self):
        """
        colony_selection calls BC_finder methods from bca_cv.py library. Then, It selects
        the colony that fits the specifications from ros topics msg.
        """

        # BC_finder parameters
        dish_size = [500,750]
        area_min = 100
        dist_col = 10
        med_filt = 11

        # Selection of colonies cooresponding to specs if operation is picking
        if self.picking:
            # BC_finder from bca_cv.py library
            [cv_image_output, perimeter, excentricity, area, color, centers] = BC_finder(self.cv_image, dish_size, area_min, dist_col, med_filt, not self.picking)

            index = np.ones(len(perimeter))

            temp_index = np.where(perimeter < self.perimeter_min)
            index[temp_index[0]] = 0

            temp_index = np.where(perimeter > self.perimeter_max)
            index[temp_index[0]] = 0

            temp_index = np.where(excentricity < self.excentricity_min)
            index[temp_index[0]] = 0

            temp_index = np.where(excentricity > self.excentricity_max)
            index[temp_index[0]] = 0

            temp_index = np.where(area < self.area_min)
            index[temp_index[0]] = 0

            temp_index = np.where(area > self.area_max)
            index[temp_index[0]] = 0

            index = (np.matrix(index)).transpose()

            color1 = (np.matrix((color[:,0]))).transpose()
            color2 = (np.matrix((color[:,1]))).transpose()
            color3 = (np.matrix((color[:,2]))).transpose()

            center1 = (np.matrix((centers[:,0]))).transpose()
            center2 = (np.matrix((centers[:,1]))).transpose()

            parameters = np.concatenate((index,perimeter,excentricity,area,color1,color2,color3,center1,center2),1)
            parameters = parameters[np.argsort(-parameters[:,0],0)][:]

            j = 0

            print(parameters.shape)

            dist = np.ones(len(perimeter)) * 1000000

            for i in parameters:
                if i[0,0] == 1 :
                    dist[j] = (i[0,6] - self.color[0])**2 + (i[0,5] - self.color[1])**2 + (i[0,4] - self.color[2])**2
                j = j + 1

            parameters = parameters[np.argsort(dist[:],0)][:]

            parameters[:,0][self.number_of_colony:] = 0

        # No selection if operation is analysis
        else:

            # BC_finder from bca_cv.py library
            [cv_image_output, perimeter, excentricity, area, color, centers] = BC_finder(self.cv_image, dish_size, area_min, dist_col, med_filt, self.picking)

            index = np.zeros(len(perimeter))
            index = (np.matrix(index)).transpose()

            color1 = (np.matrix((color[:,0]))).transpose()
            color2 = (np.matrix((color[:,1]))).transpose()
            color3 = (np.matrix((color[:,2]))).transpose()

            center1 = (np.matrix((centers[:,0]))).transpose()
            center2 = (np.matrix((centers[:,1]))).transpose()

            parameters = np.concatenate((index,perimeter,excentricity,area,color1,color2,color3,center1,center2),1)

        return parameters, cv_image_output

    def writeParamsDB(self,operation, protocol, step, parameters):
        """
        Write params to DB
        Args :
                - operation : "picking" or "analysis"
                - protocol : DB names
                - step : Protocole step
                - parameters : Parameters to write in DB
                    (x,6) numpy matrix object
        """

        # MongoDB connection
        protocol_db = self.client[protocol]

        colonies = []

        j = 0
        pixel_size = 0.0597285068

        # Parsing parameters numpy matrix to create colony dict
        for i in parameters:
            colony = {
                'operation': operation,
                'step': step,
                'id': j+1,
                'selected': parameters[j,0],
                'perimeter': parameters[j,1]*pixel_size,
                'excentricity': parameters[j,2]*pixel_size,
                'area': parameters[j,3]*(pixel_size)**2,
                'color': "#{:02x}{:02x}{:02x}".format(int(parameters[j,6]),int(parameters[j,5]),int(parameters[j,4])),
                'x': parameters[j,7]*pixel_size,
                'y': parameters[j,8]*pixel_size}

            j = j + 1
            colonies.append(colony)

        protocol_db.colonies.insert_many(colonies)

    def writeImageDB(self, operation, protocol, step, image):
        """
        Write image to DB
        Args :
                - operation : "raw", "picking" or "analysis"
                - protocol : DB names
                - step : Protocole step
                - image : image to write in DB (.jpg filename)
        """

        # MongoDB connection
        protocol_db = self.client[protocol]
        fs = GridFS(protocol_db)

        with open(image,'rb') as f:
            data = f.read()

        # Saves image to DB
        filename = "{0}_{1}.jpg".format(operation,step)
        image_id = fs.put(data, filename=filename)

        protocol_db.images.insert_one({'filename': filename, 'image_id': image_id})

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        cam = BCA()
        cam.listener()

    except (rospy.ROSInterruptException, EnvironmentError) as e:
        print(e)

