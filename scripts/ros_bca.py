#!/usr/bin/python

# import
import os
import cv2
import pprint
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as pltim
import datetime
import rospy
import sys
from scipy import ndimage as ndi
from skimage.morphology import watershed
from skimage import segmentation
from skimage import measure
from skimage import morphology
from skimage.feature import peak_local_max
from skimage.measure import regionprops

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


    def dispImg(self, img):
    	# display image
    	cv2.namedWindow("image", cv2.WINDOW_NORMAL)
    	cv2.imshow("image",img)
    	k = cv2.waitKey(0)

    def findCircularDish(self, img, rRange, factor):
	print("Im in")
    	# Function variables
    	scale = 0.1             # scale factor to find circle faster
    	radius_range = tuple([int(scale*rad) for rad in rRange])

    	# resize image (faster computation)
    	shape = img.shape
    	max_x = shape[1]
    	max_y = shape[0]
    	size = (int(scale*max_x), int(scale*max_y)) # (size_x, size_y)
    	im_scale = cv2.resize(img, dsize=size)

    	# Find circles is the image with Hough Circle Transform
    	# The algorithm returns a list of (x, y, radius) where (x, y) is center
    	circles = cv2.HoughCircles(im_scale, cv2.cv.CV_HOUGH_GRADIENT, 2, \
                 20, minRadius=radius_range[0], maxRadius=radius_range[1])

    	# Return nothing if no or more than one circle is found
    	if not isinstance(circles, type(np.empty(0))):
        	print("Error - no circle found")
        	return

    	# return data of the smallest circle found
    	circles = (circles[0, :]).astype("float")
    	mins = np.argmin(circles, axis=0)
    	indx = mins[2]
    	circle = circles[indx]

    	# Make cicular mask
    	inv_scale = 1/scale
    	c_x = np.round(inv_scale*circle[0])
    	c_y = np.round(inv_scale*circle[1])

    	nx = np.linspace(-c_x, max_x - c_x - 1, max_x)
    	ny = np.linspace(-c_y, max_y - c_y - 1, max_y)
    	mesh_x, mesh_y = np.meshgrid(nx, ny)
    	c_mask = mesh_x ** 2 + mesh_y ** 2 <= (factor*inv_scale*circle[2]) ** 2

	# Return cicular mask
	return c_mask
    
    def callback_2d_capture(self, data):
        self.flag = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        filename = "{0}.jpg".format(str(datetime.datetime.now())[:19])
        cwd = '../image_test'
        file_path = os.path.join(cwd, filename)

        cv2.imwrite(file_path, cv_image)
        print("Image {0} saved in {1}".format(filename, cwd))

	# Set path and read image
    	# image_file = "1.jpg"
    	# im_folder = os.getcwd() + "/" + "Images/"
    	#im_o = cv2.imread(im_folder + image_file)
    	
	print("test1")
	im_o = cv_image
		
	
	if im_o is None:
        	print("Requested image does not exist: {0}".format(image_file))
        	return

	print("test2")
    	im_buf = np.copy(im_o)
    	# LAB codage conversion
    	lab_image = cv2.cvtColor(im_buf, cv2.COLOR_BGR2LAB)
    	l_channel,a_channel,b_channel = cv2.split(lab_image)
	
	print("test2.1")
    	# Histogram equalization on luminance using CLAHE method
    	tileGrid = 51
    	clahe = cv2.createCLAHE(clipLimit = 10, \
        	tileGridSize=(tileGrid, tileGrid))
    	l_eq = clahe.apply(l_channel)

	print("test2.2")
    	# Locate and mask circular Petri dish
    	c_mask = self.findCircularDish(l_eq, [750, 900],.90)

	print("test2.3")
    	# Binarize image with adaptative treshold
    	nhood = 101 # size of neighbourhood
    	offset = 20
    	BW1 = cv2.adaptiveThreshold(l_eq,255,\
        	cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
        	cv2.THRESH_BINARY_INV,nhood,offset)

	print("test2.3.1")
    	# Apply median blur to smooth edges and remove noise
    	BW1 = cv2.medianBlur(BW1, 11)

	print("test2.3.2")
    	# Apply circular mask
    	idx = (c_mask== False)
    	BW2 = np.copy(BW1)
    	BW2[idx] = 0;
	
	print("test2.4")
    	# Watershed
    	dist_col = 5

    	distance = ndi.distance_transform_edt(BW2)
    	local_maxi = peak_local_max(distance, indices=False,\
        	footprint=np.ones((dist_col, dist_col)), labels=BW2)
    	markers = morphology.label(local_maxi)
    	labels = watershed(-distance, markers, mask=BW2)

    	# Extract areas from regions
    	props = regionprops(labels)
	print("test2.5")  
    	# Remove small regions
    	area_min = 100
    	for val in props:
        	if int(val.area) < area_min:
            		idx = (labels == val.label)
            		BW2[idx] = 0

        print("test3")

    	# Extract info from new regions
    	distance = ndi.distance_transform_edt(BW2)
    	local_maxi = peak_local_max(distance, indices = False,\
        	footprint=np.ones((dist_col, dist_col)), labels = BW2)
    	markers = morphology.label(local_maxi)
    	labels = watershed(-distance, markers, mask = BW2)
    	props = regionprops(labels)

    	centers = [[int(round(i)) for i in val.centroid] for val in props]
    	bbox = [[int(round(i)) for i in val.bbox] for val in props]

    	contours = segmentation.find_boundaries(BW2)


     	# Show results
    	im_f = cv_image
    	im_f = segmentation.mark_boundaries(im_f, labels)
	
	print("test4")
	#for val in bbox:
        	#cv2.rectangle(im_o, (val[1], val[0]), (val[3], val[2]), \
        	#    (255, 0, 0),thickness=3)

    	for val in centers:
        	cv2.circle(im_f, (val[1], val[0]), 2, (0, 255, 0), -1)
    	
	print("test1")
	fig2 = plt.figure()

    	plt.imshow(im_f)
    	plt.show()
    	pltim.imsave('output.pdf', im_f, format='pdf')
	
	cv2.imwrite(file_path, im_f)
	print("Analysis done ...")


    def listener(self):
        while not self.flag:
            rospy.sleep(100)

if __name__ == '__main__':
    try:
        cam = Camera2D()
        cam.listener()

    except (rospy.ROSInterruptException, EnvironmentError) as e:
        print(e)

