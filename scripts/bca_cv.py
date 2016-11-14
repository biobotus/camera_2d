#!/usr/bin/env python
__author__ = "Jean-Samuel Lauzon, Etienne Pelletier, Marc-Antoine Deragon"
__copyright__ = ""
__credits__ = ["Jean-Samuel Lauzon", "Etienne Pelletier",
	"Marc-Antoine Deragon"]
__license__ = ""
__version__ = "1.0.1"
__maintainer__ = ""
__email__ = "jean-samuel.lauzon@usherbrooke.ca"
__status__ = "Dev"

# import
import os
import cv2
import pprint
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage as ndi
from skimage.morphology import watershed
from skimage import segmentation
from skimage import measure
from skimage import morphology
from skimage.feature import peak_local_max
from skimage.measure import regionprops


def dispImg(img):
    # display image
    cv2.namedWindow("image", cv2.WINDOW_NORMAL)
    cv2.imshow("image",img)
    k = cv2.waitKey(0)
    
def findCircularDish(img, rRange, factor):
    
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
    

def BC_finder(im_o, dishSize,  area_min, dist_col, med_filt):

    # LAB codage conversion
    lab_image = cv2.cvtColor(im_o, cv2.COLOR_BGR2LAB)
    l_channel,a_channel,b_channel = cv2.split(lab_image)
    
    # Histogram equalization on luminance using CLAHE method
    clahe = cv2.createCLAHE(clipLimit = 9.0, tileGridSize=(11,11))
    l_eq = clahe.apply(l_channel)
    
    # Locate and mask circular Petri dish
    c_mask = findCircularDish(l_eq, dishSize,.90)
    
    # Binarize image with adaptative treshold
    nhood = 101 # size of neighbourhood
    offset = 20
    BW1 = cv2.adaptiveThreshold(l_eq,255,\
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
        cv2.THRESH_BINARY_INV,nhood,offset)
        
    # Apply median blur to smooth edges and remove noise
    BW1 = cv2.medianBlur(BW1, med_filt)
    
    # Apply circular mask
    idx = (c_mask== False)
    BW2 = np.copy(BW1)
    BW2[idx] = 0;
    
    # Watershed
    distance = ndi.distance_transform_edt(BW2)
    local_maxi = peak_local_max(distance, indices=False,\
        footprint=np.ones((dist_col, dist_col)), labels=BW2)
    markers = morphology.label(local_maxi)
    labels = watershed(-distance, markers, mask=BW2)
    
    # Extract areas from regions
    props = regionprops(labels)
  
    # Remove small regions
    for val in props:
        if int(val.area) < area_min:
            idx = (labels == val.label)
            BW2[idx] = 0
    
    # Redo watershed on new section
    distance = ndi.distance_transform_edt(BW2)
    local_maxi = peak_local_max(distance, indices = False,\
        footprint=np.ones((dist_col, dist_col)), labels = BW2)
    markers = morphology.label(local_maxi)
    labels = watershed(-distance, markers, mask = BW2)
    props = regionprops(labels)
    
    # Extract info from new regions
    centers = [[int(round(i)) for i in val.centroid] for val in props]
    bbox = [[int(round(i)) for i in val.bbox] for val in props]

    contours = segmentation.find_boundaries(BW2)
    
    # Generate new images with bounding boxes

    #for val in bbox:
    #    cv2.rectangle(im_o, (val[1], val[0]), (val[3], val[2]), (255, 0, 0),thickness=3)
    #for val in centers:    
    #    cv2.circle(im_o, (val[1], val[0]), 2, (0, 255, 0), -1)
    
    # Show results
    im_o = segmentation.mark_boundaries(im_o, labels)
    return im_o
    
def main():
# Set path and read image
    image_file = "1.jpg"
    im_folder = os.getcwd() + "/" + "Images/"
    im_o = cv2.imread(im_folder + image_file)
    
    if im_o is None:
        print("Requested image does not exist: {0}".format(image_file))
        return
        
    im_f = BC_finder(im_o, [750, 900],  100, 10, 7)
    fig2 = plt.figure()
    plt.imshow(im_f)
    plt.show()
if __name__ == "__main__":
    main()
