#!/usr/bin/python

import cv2

# Colors (BGR)
COLOR_BLACK = (0,   0,   0)
COLOR_WHITE = (255, 255, 255)
COLOR_BLUE  = (255, 0,   0)
COLOR_GREEN = (0,   255, 0)
COLOR_RED   = (0,   0,   255)

# Size of printed pictures (biggest side, in pixels)
PIC_SIZE = 500.

def imshow(img_dict):
    """
    Prints multiple images at once and closes them on keystroke.
    More user-friendly than using cv2.imshow()
    Note : image is rotated automatically so the bigger side is horizontal

    Inputs:
        - img_dict : dictionnary of images. Keys are the names, values are cv2 images
            --> {"name": image} can be used if only one to print
    """

    if not isinstance(img_dict, dict):
        print("Invalid data type in imshow. Function doc:\n{0}".format(imshow.__doc__))
        return

    try:
        for img in img_dict.keys():
            shape = img_dict[img].shape
            scale = PIC_SIZE/max(shape)
            size = (int(scale*shape[1]), int(scale*shape[0])) # (size_x, size_y)
            cv2.imshow(img, cv2.resize(img_dict[img], dsize=size))

    except cv2.error as e:
        print(e)

    finally:
        cv2.waitKey(0)
        cv2.destroyAllWindows()

