#!/usr/bin/env python
import rospy

import numpy as np

import cv2
import cv2.cv as cv
import cv_bridge

from sensor_msgs.msg import Image
from domino_coord.srv import ImageSrv, ImageSrvResponse, DominoCoordSrv, DominoCoordSrvResponse, DominoCoordSrvRequest


if __name__ == '__main__':
  rospy.wait_for_service('domino_finder')
  rospy.init_node('image_processing_node')
  img_analyze_service = rospy.ServiceProxy('domino_finder', DominoCoordSrv)
  d= img_analyze_service(1000,[50,100],[10,50,20,0,35])
  print d.dominos
  cv2.destroyAllWindows()
