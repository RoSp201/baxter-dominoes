#!/usr/bin/env python
import rospy

import numpy as np

import cv2
import cv2.cv as cv
import cv_bridge

from sensor_msgs.msg import Image
from domino_coord.srv import ImageSrv, ImageSrvResponse, DominoCoordSrv, DominoCoordSrvResponse


SAMPLE_LENGTH = 100

class DominoService:
  def __init__(self):
    #Create an instance variable to store the last image received
    self.lastImage = None;

    #Initialize the node
    rospy.init_node('cam_listener')

    #Subscribe to the image topic
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.imgReceived)

    #Create last image publisher
    rospy.Publisher('baxter_image', ImageSrv)

    #Create analyze image service
    rospy.Service('domino_finder', DominoCoordSrv, self.analyzeMultipleImages)

    print "All Ready"

  def imgReceived(self, message):
    self.lastImage = message
    

  def analyzeImage(self, request):
    # Collect CV params
    contourArea, cParams, hParams = request.contourArea, request.cannyParams, request.houghParams
    # Initialize response array
    response = []

    # Convert image to mat, grayscale, blur , and apply canny filter
    orig = cv_bridge.CvBridge().imgmsg_to_cv2(self.lastImage, desired_encoding="passthrough")
    gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3),0)
    img = cv2.Canny(blur,cParams[0],cParams[1])


    # Find domino contours
    contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_TC89_KCOS )
    dominoContour = [cont for cont in contours if cv2.contourArea(cont)>contourArea]

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
        cv2.drawContours(orig,[box],0,(46,97,87),10)
        cv_bridge.CvBridge().cv2_to_imgmsg(orig, encoding="passthrough")

        # Transform Domino images
        pts1 = np.float32([box[0],box[1],box[2],box[3]])
        pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dom = cv2.warpPerspective(blur,M,(int(w),int(h)))

        # Check orientation
        vert = False
        if dom.shape[0] > dom.shape[1]:
            vert = True

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
        # Add results to array
        response += [side1, side2, vert, cx, cy]

    return response

  def analyzeMultipleImages(self, request):
    # Init
    totalResponse = self.analyzeImage(request)
    # Use multiple samples
    for i in range(SAMPLE_LENGTH):
      # Do a sample
      response = self.analyzeImage(request)
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