#!/usr/bin/env python
import rospy

import numpy as np

import cv2
import cv2.cv as cv
import cv_bridge

from sensor_msgs.msg import Image
from domino_coord.srv import ImageSrv, ImageSrvResponse, DominoCoordSrv, DominoCoordSrvResponse

BUFFER_LENGTH = 100

class DominoService:
  imgBuffer = []
  counter = -1

  def imgReceived(self, message):
    if counter == -1:
      imgBuffer = [message] * BUFFER_LENGTH
      counter = 0
    else:
      imgBuffer[0] = message
      counter = (counter + 1) % BUFFER_LENGTH


  def getLastImage(self, request):
    return ImageSrvResponse(self.lastImage)

  def analyzeImage(self, request):
    contourArea, cParams, hParams = request.contourArea, request.cannyParams, request.houghParams
    response = []

    orig = cv_bridge.CvBridge().imgmsg_to_cv2(self.lastImage, desired_encoding="passthrough")
    gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3),0)
    img = cv2.Canny(blur,cParams[0],cParams[1])

    #img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    contours, hierarchy = cv2.findContours(img,1,cv2.CHAIN_APPROX_SIMPLE)
    cont2 = [cont for cont in contours if cv2.contourArea(cont)>contourArea]
    for i in range(len(cont2)):
        ((cx,cy),(w,h),angle) = rect = cv2.minAreaRect(cont2[i])
        match = False
        for i in range(len(response)5):
          if abs(cx-response[3 + 5*i]) < 10 or abs(cy - response[4 + 5*i])< 10:
            match = True
        if match:
          continue
        if abs(angle) > 45:
            w,h = h,w
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        box = np.array(sorted(box, key=lambda pt: pt[0]+pt[1]))
        if box[1][0] < box[2][0]:
            temp = box[1].copy()
            box[1] = box[2]
            box[2] = temp
        pts1 = np.float32([box[0],box[1],box[2],box[3]])
        pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dom = cv2.warpPerspective(blur,M,(int(w),int(h)))
        vert = False
        if dom.shape[0] > dom.shape[1]:
            vert = True

        circles = cv2.HoughCircles(dom, cv.CV_HOUGH_GRADIENT,1,hParams[0],param1=hParams[1],param2=hParams[2],minRadius=hParams[3],maxRadius=hParams[4])
        side1 = 0
        side2 = 0
        if circles != None:
            circles = np.uint16(np.around(circles))
            for j in circles[0,:]:
                # draw the outer circle
                cv2.circle(dom,(j[0],j[1]),j[2],(100,255,100),2)
                # draw the center of the circle
                cv2.circle(dom,(j[0],j[1]),2,(100,100,255),3)
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
        response += [side1, side2, vert, cx, cy]
    return response

  def analyzeBuffer(self, request):
    totalResponse = []
    for i in range(BUFFER_LENGTH):
      response = self.analyzeImage(request)
      for i in range(len(response) / 5):
        for i in range(len(totalResponse) / 5):
          if abs(response[3 + 5*i])


  def __init__(self):
    #Create an instance variable to store the last image received
    self.lastImage = None;

    #Initialize the node
    rospy.init_node('cam_listener')

    #Subscribe to the image topic
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.imgReceived)

    #Create last image service
    rospy.Service('last_image', ImageSrv, self.getLastImage)

    #Create analyze image service
    rospy.Service('domino_finder', DominoCoordSrv, self.analyzeBuffer)

    print "All Ready"

  def run(self):
    rospy.spin()

if __name__ == '__main__':
  node = DominoService()
  node.run()