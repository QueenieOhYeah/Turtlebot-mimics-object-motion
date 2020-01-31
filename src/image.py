#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import roslib
roslib.load_manifest('turtlesim_move')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from turtlesim_move.msg import Cen_pose
from math import pow, sqrt

import numpy as np
from collections import deque

def nothing(*arg):
    pass
pts = deque(maxlen = 10)

class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("center_position",Cen_pose, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      height, width = cv_image.shape[:2]
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # Get value from the track bar

    lowHue = cv2.getTrackbarPos('lowHue', 'colorTest')
    lowSat = cv2.getTrackbarPos('lowSat', 'colorTest')
    lowVal = cv2.getTrackbarPos('lowVal', 'colorTest')
    highHue = cv2.getTrackbarPos('highHue', 'colorTest')
    highSat = cv2.getTrackbarPos('highSat', 'colorTest')
    highVal = cv2.getTrackbarPos('highVal', 'colorTest')
    frameHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # HSV values to define a colour range we want to create a mask from.
    colorLow = np.array([lowHue,lowSat,lowVal])
    colorHigh = np.array([highHue,highSat,highVal])
    mask = cv2.inRange(frameHSV, colorLow, colorHigh)

    # Show the first mask
    cv2.imshow('mask-plain', mask)

    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

    #cv2.drawContours(frame, biggest_contour, -1, (0,255,0), 3)

    x,y,w,h = cv2.boundingRect(biggest_contour)
    center = (int(x+w/2),int(y+h/2))
    #center = (5,5)
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
      # if either of the tracked points are None, ignore
      # them
      if pts[i - 1] is None or pts[i] is None:
    	continue

      # otherwise, compute the thickness of the line and
      # draw the connecting lines
      thickness = int(np.sqrt(4 / float(i + 1)) * 2.5)
      cv2.line(cv_image, pts[i - 1], pts[i], (0, 0, 255), thickness)

    #cv2.circle(cv_image, center, 1, (0, 0, 255), -1)
    cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)

    # Show final output image
    cv2.imshow('colorTest', cv_image)

    k = cv2.waitKey(5) & 0xFF

    try:
      center_position = Cen_pose()
      center_position.x = center[0]/width * 10
      center_position.y = 10 - center[1]/height * 10
      #if sqrt(pow((pts[0][0] - pts[len(pts)-1][0]), 2) + pow((pts[0][1] - pts[len(pts)-1][1]), 2)) > 20:
      self.image_pub.publish(center_position)
    except CvBridgeError as e:
      print(e)

def main(args):
  #icol = (104, 117, 222, 121, 255, 255)   # test
  #icol = (18, 0, 196, 36, 255, 255)  # Yellow
  #icol = (0, 100, 80, 10, 255, 255)   # Red
  #icol = (0, 193, 63, 38, 255, 255)   # My Red
  icol = (0, 124, 65, 255, 255, 255)
  #icol = (0, 0, 0, 255, 255, 255)   # New start

  cv2.namedWindow('colorTest')
  # Lower range colour sliders.
  cv2.createTrackbar('lowHue', 'colorTest', icol[0], 255, nothing)
  cv2.createTrackbar('lowSat', 'colorTest', icol[1], 255, nothing)
  cv2.createTrackbar('lowVal', 'colorTest', icol[2], 255, nothing)
  # Higher range colour sliders.
  cv2.createTrackbar('highHue', 'colorTest', icol[3], 255, nothing)
  cv2.createTrackbar('highSat', 'colorTest', icol[4], 255, nothing)
  cv2.createTrackbar('highVal', 'colorTest', icol[5], 255, nothing)

  ic = image_converter()
  #rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
