#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image
import cv_bridge

first = True

def imgReceived(message):
    print "______________________________"
    img = cv_bridge.CvBridge().imgmsg_to_cv2(message, desired_encoding="passthrough")
    contourArea = cv2.getTrackbarPos('contourArea','settings')
    canny1 = cv2.getTrackbarPos('canny1','settings')
    canny2 = cv2.getTrackbarPos('canny2','settings')

    minDist = cv2.getTrackbarPos('minDist','settings')
    param1 = cv2.getTrackbarPos('param1','settings')
    param2 = cv2.getTrackbarPos('param2','settings')
    minRadius = cv2.getTrackbarPos('minRadius','settings')
    maxRadius = cv2.getTrackbarPos('maxRadius','settings')

    orig = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3),0)
    img = cv2.Canny(blur,canny1,canny2)
    response = []
    #img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    contours, hierarchy = cv2.findContours(img,1,cv2.CHAIN_APPROX_SIMPLE)
    cont2 = [cont for cont in contours if cv2.contourArea(cont)>contourArea]
    for i in range(len(cont2)):
        ((cx,cy),(w,h),angle) = rect = cv2.minAreaRect(cont2[i])
        match = False
        for i in range(len(response)/5):
          if abs(cx-response[3 + 5*i]) < 10 or abs(cy - response[4 + 5*i])< 10:
            match = True
        if match:
          continue
        if abs(angle) > 45:
            w,h = h,w
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img,[box],0,(100,100,255),10)
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

        circles = cv2.HoughCircles(dom, cv.CV_HOUGH_GRADIENT,1,minDist,param1=param1,param2=param2,minRadius=minRadius,maxRadius=maxRadius)
        side1 = 0
        side2 = 0
        if circles != None:
            circles = np.uint16(np.around(circles))
            print("domino" + str(i))
            print("no. circles: " + str(len(circles[0,:])))
            print(dom.shape)
            print ""
            for j in circles[0,:]:
                print (j[0],j[1])
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
        print(str(side1) + " | " + str(side2))
        print (cx, cy)
        print vert
        response += [side1, side2, vert, cx, cy]
        cv2.imshow('domino' + str(i), dom)


    #cv2.drawContours(img, cont2, 1, (100,100,100), 10)
    cv2.imshow('img',img)
    cv2.waitKey(1)


def listener():
    cv2.namedWindow('settings')

    cv2.createTrackbar('contourArea','settings',0,5000,nothing)

    cv2.createTrackbar('canny1','settings',0,300,nothing)
    cv2.createTrackbar('canny2','settings',0,500,nothing)

    cv2.createTrackbar('minDist','settings',0,20,nothing)   
    cv2.createTrackbar('param1','settings',0,100,nothing)
    cv2.createTrackbar('param2','settings',0,50,nothing)
    cv2.createTrackbar('minRadius','settings',0,20,nothing)
    cv2.createTrackbar('maxRadius','settings',0,50,nothing)

    cv2.setTrackbarPos('contourArea','settings', 1000)

    cv2.setTrackbarPos('canny1','settings', 50)
    cv2.setTrackbarPos('canny2','settings', 100)

    cv2.setTrackbarPos('minDist','settings', 10)
    cv2.setTrackbarPos('param1','settings', 50)
    cv2.setTrackbarPos('param2','settings', 20)
    cv2.setTrackbarPos('minRadius','settings', 0)
    cv2.setTrackbarPos('maxRadius','settings', 35)

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, imgReceived)
    #rospy.Subscriber("/usb_cam/image_raw", Image, imgReceived)
    rospy.spin()

def nothing(nothing):
    pass

if __name__ == '__main__':
    listener()