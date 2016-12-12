#!/usr/bin/env python
import rospy

import numpy as np

import cv2
import cv2.cv as cv
import cv_bridge

from sensor_msgs.msg import Image
from follow.srv import *

SAMPLE_LENGTH = 100
COUNTER_LENGTH = 3

class DominoService:
  def __init__(self):
    #Create an instance variable to store the last image received
    self.lastImage = None;

    #Initialize the node
    rospy.init_node('cam_listener')

    #Subscribe to the image topic
    #rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.imgReceived)
    rospy.Subscriber("/usb_cam/image_raw", Image, self.imgReceived)

    #Create last image publisher
    self.img_pub = rospy.Publisher('baxter_image', Image, queue_size=10)

    #Create analyze image service
    rospy.Service('domino_finder', DominoCoordSrv, self.analyzeMultipleImages)

    self.counter = 0
    self.copy = None

    print "All Ready"

  def imgReceived(self, message):
    self.lastImage = message
    self.analyzeImage([50000, [40, 50], [10, 50, 20, 0 ,35]])

  def analyzeImage(self, params):
    # Collect CV params
    contourArea, cParams, hParams = params
    # Initialize response array
    response = []

    # Convert image to mat, grayscale, blur , and apply canny or threshold filter
    orig = cv_bridge.CvBridge().imgmsg_to_cv2(self.lastImage, desired_encoding="passthrough")
    cv2.imwrite( "../img.jpg", orig);
    if self.counter == 0:
        self.copy = orig.copy()
    gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3),0)
    img = cv2.Canny(blur,cParams[0],cParams[1])
    #img = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    if self.counter == 0:
        self.copy = cv2.cvtColor(img.copy(), cv2.COLOR_GRAY2BGR)

    # Find domino contours
    contours, hierarchy = cv2.findContours(img,1,cv2.CHAIN_APPROX_SIMPLE)
    dominoContour = []
    for i in range(len(contours)):
        ((cx,cy),(w,h),angle) = rect = cv2.minAreaRect(contours[i])
        # THESE MIGHT NEED TO BE TUNED!
        if w*h < 10000000 and w*h > 10000 and (abs(2*w - h) > 20 or abs(2*h - w) > 20) and (abs(abs(angle) - 90) < 20 or abs(angle) < 20) :
            dominoContour += [contours[i]]
    #dominoContour = [cont for cont in contours if cv2.contourArea(cont)>contourArea and cv2.contourArea(cont)<100000]

    # For each domino
    for i in range(len(dominoContour)):
        # Fit a rectangle to dominos
        ((cx,cy),(w,h),angle) = rect = cv2.minAreaRect(dominoContour[i])

        # For duplicate robustness check for dominoes eerily close to each other
        match = False
        for i in range(len(response)/5):
          if abs(cx-response[3 + 5*i]) < 10 or abs(cy - response[4 + 5*i])< 10:
            match = True
        if match:
          continue

        # Find rotation (this is occasinally wonky)
        if abs(angle) > 45:
            w,h = h,w

        # Find coordinates and put in reasonable order
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        box = np.array(sorted(box, key=lambda pt: pt[0]+pt[1]))
        if box[1][0] < box[2][0]:
            temp = box[1].copy()
            box[1] = box[2]
            box[2] = temp

        # Publish cool looking image hopefully
        cv2.drawContours(self.copy,dominoContour,i,(255,0,0),10)

        # Transform Domino images
        pts1 = np.float32([box[0],box[1],box[2],box[3]])
        pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dom = cv2.warpPerspective(blur,M,(int(w),int(h)))

        # Check orientation
        vert = False
        if dom.shape[0] > dom.shape[1]:
            vert = True
        cv2.imshow('img', dom)
        # Start countin' circles!
        circles = cv2.HoughCircles(dom, cv.CV_HOUGH_GRADIENT,1,hParams[0],param1=hParams[1],param2=hParams[2],minRadius=hParams[3],maxRadius=hParams[4])
        side1 = 0
        side2 = 0
        if circles != None:
            circles = np.uint16(np.around(circles))
            for j in circles[0,:]:
                if not vert:
                    if j[0] <= dom.shape[1]/2:
                        side1 += 1
                    else:
                        side2 += 1
                else:
                    if j[1] <= dom.shape[0]/2:
                        side1 += 1
                    else:
                        side2 += 1
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.copy,str(side1) + " | " + str(side2),(int(cx), int(cy)), font, 1,(255,0,255),2)
        # Add results to array
        response += [side1, side2, vert, cx, cy]

    self.counter = (self.counter + 1) % COUNTER_LENGTH
    if self.counter == COUNTER_LENGTH - 1:
        self.img_pub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(self.copy, encoding="passthrough"))
    return response

  def analyzeMultipleImages(self, request):
    params = [request.contourArea, request.cannyParams, request.houghParams]
    totalResponse = self.analyzeImage(params)
    # Use multiple samples
    for i in range(SAMPLE_LENGTH):
      # Do a sample
      response = self.analyzeImage(params)
      # Check each domino response
      for j in range(len(response) / 5):
        # Compare to each domino is current accumlation of dominos
        for k in range(len(totalResponse) / 5):
          # Check for same domino, take max of dots since they occasionally drop
          if abs(response[3 + 5*j] - totalResponse[3 + 5*k]) < 10:
            totalResponse[5 * k] = max(totalResponse[5 * k], response[5*j])
            totalResponse[1 + 5 * k] = max(totalResponse[1 + 5 * k], response[1 + 5*j])
            
    return DominoCoordSrvResponse(totalResponse)

  def run(self):
    rospy.spin()

if __name__ == '__main__':
  node = DominoService()
  node.run()
