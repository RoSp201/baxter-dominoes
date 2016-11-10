#!/usr/bin/env python

import sys
import rospy
from DominoMove.srv import *

def domino_client(x, y):
    rospy.wait_for_service('domino_move')
    try:
        get_best_move = rospy.ServiceProxy('domino_move', DominoMove)
        resp1 = get_best_move(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        # INPUT TEST DOMINOES HERE
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, domino_client(x, y))
