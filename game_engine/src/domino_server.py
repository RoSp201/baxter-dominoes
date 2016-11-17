#!/usr/bin/env python

from DominoMove.srv import *
import dominoes
import rospy

def handle_domino_move(req):
    # DO game computation stuff here
    print "IT WOULD BE COOL IF THIS WORKED"
    for tup in req.board:
    return DominoMoveResponse(

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('domino_move', GameState, handle_domino_move)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
