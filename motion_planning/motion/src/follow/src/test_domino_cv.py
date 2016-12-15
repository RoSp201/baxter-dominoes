#!/usr/bin/env python
import rospy

import numpy as np

import cv2
import cv2.cv as cv
import cv_bridge

from sensor_msgs.msg import Image
from follow.srv import *

def main():
    """Tests output of domino classification cv analysis in cv_srv.py"""
    rospy.init_node('image_processing_node')
    rospy.wait_for_service('domino_finder')
    print "Found Service"
    img_analyze_service = rospy.ServiceProxy('domino_finder', DominoCoordSrv)
    d = img_analyze_service(1000,[50,60],[10,50,20,0,35])
    print "Number of Dominos: " + str(len(d.dominos))
    print "_____"
    print d.dominos
    print " "
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
  