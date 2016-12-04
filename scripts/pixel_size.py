#!/usr/bin/env python

import cv2
import zbar
import time

def scan(img, cam):
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    raw = str(imgray.data)

    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')

    width = int(cam.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
    height = int(cam.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
    imageZbar = zbar.Image(width, height,'Y800', raw)
    scanner.scan(imageZbar)

    tmp = [item.location for item in imageZbar]
    return tmp[0] if tmp else []

def pixel_size(qr_size, points):
    dist = []

    print(points)

    for i in [points[0:2], points[1:3], points[2:4], points[::3]]:
        dist.append(((i[0][0]-i[1][0])**2+(i[0][1]-i[1][1])**2)**0.5)
    return qr_size/sum(dist)*4

cam = cv2.VideoCapture(0)
points = None
while not points:
    ret, img = cam.read()
    points = scan(img,cam)
    print('.')

qr_dimension = 52.8

print(points)
print(pixel_size(qr_dimension, points))
for i in points:
    cv2.circle(img, i, 5, (0,0,255), -1)

cv2.imwrite('image.jpg',img)
