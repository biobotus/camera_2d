# ros_bca

This repository contains all the script needed to used the BCA (bacterial colony analysis) 
module of BioBotus project.

It contains a ros node for bacterial colony analysis (ros_bca.py), 
a python algorithm for bacterial analysis (bca_cv.py) and a script to find the 
pixel_size of an image at a fix distance for a 2D camera using QR codes. (pixel_size.py)


----------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------

- ros_bca.py 
    This ROS node controls the bca.
    It is subscribed to BC_Analysis that send characteristics
        -picking (bool) true if it is a repicking false if it is a simple analysis
        -protocol (string) where to write informations in DB
        -step (int32) which step in protocol
        -pick_number (int32) which pick number for multi repicking purposes

    If picking is True it also send characteristics to find in colony:
        -perimeter_min (float) 
        -perimeter_max (float)
        -excentricity_min (float)
        -excentricity_max (float)
        -area_min (float)
        -area_max (float)
        -number_of_colony (int32)
        -color (string) hexa color value

    These characteristics are used for the classification done in this ROS node.

    Once the colony are classified or not in case of a simple analysis, there informations
    are written in the BioBot DB with the informations given by the BC_Analysis ROS topic.  
   
    It also subscribed to raw_image which is a ROS topics sent by uvc_camera ROS nodes. The
    msg is an imgmsg which is converted to a 'bgr8' image with openCV method bridge.imgmsg_to_cv2()
    and saved in the global variable self.cv_image of the ros_bca class.

    It publish a Done_Module("BC_Analysis") once the analysis is done.

    Note : Once a BC_Analysis is received, if an exception is thrown during the analysis, the script
           will retry the analysis. If it failed 4 times, the script will publish a Done_Module
           and the message 'Impossible to finalize BC Analysis ...' will be printed.

------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------

- bca_cv.py 
    Python algorithm using openCV to analyze bacterial colony. This algo. is used by ros_bca 
    to find the characteritics of every bacterial colony in the given image of a petri dish.

    This script is stand alone, it can be used to count and characterize bacterial colony 
    on a 2D image of a petri dish independently of BioBotus project.

    For further informations on this algorithm, see the user guide at : 

------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------    
        
- pixel_size.py        
    Python script to find the pixel size of an image at a fix distance of a 2D camera using QR
    code. It is a stand alone script. The value of the QR code width in mm needs to be hardcoded 
    in the script. Once the script is running, it takes the image flux and return a pixel size
    value once it finds the QR code in the camera field of view. 

    This value then needs to be hardcoded in the ros_bca.py ROS node. 

    This methods only works for non fish-eye 2D camera with a low radial distortion of the lens
    and with a small distance between the camera and the image position. 

    Note : This script is still development and is not working yet.

------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------
