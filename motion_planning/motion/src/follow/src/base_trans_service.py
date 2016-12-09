import rospy
import tf
import numpy as np

def handle_translate(coords):
    #this function will turn camera coordinates into base frame coordinates 
    pass

def translate_server():
    rospy.init_node("world_trans_server")
    s = rospy.Service("world_trans_server", Translate, handle_translate)
    print "coordinate translator server ready to use!" 
    rospy.spin()

if __name__ == "__main__":
    translate_server()

