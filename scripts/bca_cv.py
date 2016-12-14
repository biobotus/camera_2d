#!/usr/bin/python
__author__ = "Jean-Samuel Lauzon, Etienne Pelletier, Marc-Antoine Deragon"
__copyright__ = ""
__credits__ = ["Jean-Samuel Lauzon", "Etienne Pelletier", "Marc-Antoine Deragon"]
__license__ = ""
__version__ = "1.0.1"
__maintainer__ = ""
__email__ = "jean-samuel.lauzon@usherbrooke.ca"
__status__ = "Dev"



# import
import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage as ndi

from skimage import segmentation
from skimage import measure
from skimage import morphology
from skimage.feature import peak_local_max
from skimage.measure import regionprops
from skimage import measure
from skimage.morphology import watershed


def findCircularDish(img, rRange, factor):
    # Function variables
    scale = 0.1  # scale factor to find circle faster
    radius_range = tuple([int(scale*rad) for rad in rRange])

    # resize image (faster computation)
    shape = img.shape
    max_x = shape[1]
    max_y = shape[0]
    size = (int(scale*max_x), int(scale*max_y))  # (size_x, size_y)
    im_scale = cv2.resize(img, dsize=size)


    #hold the image to help find circles
    ret,th1 = cv2.threshold(im_scale,100,255,cv2.THRESH_BINARY)
    
    # Find circles is the image with Hough Circle Transform
    # The algorithm returns a list of (x, y, radius) where (x, y) is center
    circles = cv2.HoughCircles(th1, cv2.cv.CV_HOUGH_GRADIENT, 2, \
                 200, minRadius=radius_range[0], maxRadius=radius_range[1])

    # Return nothing if no or more than one circle is found
    if not isinstance(circles, type(np.empty(0))):
        print("Error - no circle found")
        raise Exception('Error - no circle found')
        return

    # Return data of the smallest circle found
    circles = (circles[0, :]).astype("float")
    max_c = np.argmax(circles, axis=0)
    indx = max_c[2]
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
    return c_mask, circle*inv_scale

def BC_finder(im_o, dishSize,  area_min, dist_col, med_filt, use_watershed=True):
    # LAB codage conversion
    lab_image = cv2.cvtColor(im_o, cv2.COLOR_BGR2LAB)
    l_channel,a_channel,b_channel = cv2.split(lab_image)

    # Histogram equalization on luminance using CLAHE method
    clahe = cv2.createCLAHE(clipLimit = 9.0, tileGridSize=(11,11))
    l_eq = clahe.apply(l_channel)

    # Locate and mask circular Petri dish
    c_mask, dish = findCircularDish(l_eq, dishSize,.90)

    # Binarize image with adaptative treshold
    nhood = 151  # size of neighbourhood
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

    if use_watershed:
        # Watershed
        distance = ndi.distance_transform_edt(BW2)
        local_maxi = peak_local_max(distance, indices=False,\
            footprint=np.ones((dist_col, dist_col)), labels=BW2)
        markers = morphology.label(local_maxi)
        labels = watershed(-distance, markers, mask=BW2)

        # Extract areas from regions
        props = regionprops(labels)

        # Remove small regions and find color
        for val in props:
            if int(val.area) < area_min:
                idx = (labels == val.label)
                BW2[idx] = 0

        # Redo watershed on new sections
        distance = ndi.distance_transform_edt(BW2)
        local_maxi = peak_local_max(distance, indices = False,\
            footprint=np.ones((dist_col, dist_col)), labels = BW2)
        markers = morphology.label(local_maxi)
        labels = watershed(-distance, markers, mask = BW2)
    else:
        label_objects, nb_labels = ndi.label(BW2)
        sizes = np.bincount(label_objects.ravel())
        mask_sizes = sizes > area_min
        mask_sizes[0] = 0
        BW3 = mask_sizes[label_objects]
        labels = morphology.label(BW3, background=0)

    # Extract info from new regions
    props = regionprops(labels)
    N = len(props)
    colors = np.zeros([N,3], dtype=int)
    centers = np.zeros([N,2], dtype=int)
    bboxes = np.zeros([N,4], dtype=int)
    areas = np.zeros([N,1])
    perimeters = np.zeros([N,1])
    eccentricities = np.zeros([N,1])
    for val in props:
        # Extract center positions
        centers[val.label-1,:] = [i for i in val.centroid]
        # Extract perimeters
        perimeters[val.label-1] = val.perimeter
        # Extract areas
        areas[val.label-1] = val.area
        # Extract eccentricity
        eccentricities[val.label-1] = val.eccentricity
        # Extract bounding box positions
        bboxes[val.label-1,:] = [int(i) for i in val.bbox]
        # Extract colors
        idx = (labels == val.label)
        colors[val.label-1,:] = np.round(np.mean(im_o[idx,:],axis=0))


    # Generate new images with centers
    for val in centers:
        cv2.circle(im_o, (int(val[1]), int(val[0])), 3, (0, 255, 0), -1)
    #for val in bboxes:
        #cv2.rectangle(im_o, (val[1],val[0]), (val[3],val[4]), (0,255,0), thickness=1, lineType=8, shift=0)
    
    cv2.circle(im_o, (int(dish[0]), int(dish[1])), int(dish[2]), (0, 255, 0), 10)

    dim_im = im_o.shape
    centers[:,0] = centers[:,0] - int(dim_im[0]/2)
    centers[:,1] = centers[:,1] - int(dim_im[1]/2)

    im_output = im_o

    # Return values
    return im_output, perimeters, eccentricities, areas, colors, centers


if __name__ == "__main__":
    # Set path and read image
    image_file = "1.1.jpg"
    im_o = plt.imread(image_file)

    if im_o is None:
        print("Requested image does not exist: {0}".format(image_file))
        quit()

    im_output, perimeters, eccentricities, areas, colors, centers = \
        BC_finder(im_o, [800,900],  100, 10, 7, use_watershed=False)

    cv2.imwrite("2.jpg",im_output)
    print(centers)

    plt.figure()
    plt.imshow(im_output)
    plt.show()
