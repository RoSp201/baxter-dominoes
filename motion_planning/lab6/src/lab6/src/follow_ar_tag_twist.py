#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf

from ar_tag_subs import compute_twist, return_rbt

listener = None

def follow_ar_tag(zumy, ar_tags):

    #create listener node for transform
    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher('%s/cmd_vel' % zumy, Twist, queue_size=2)
    rate = rospy.Rate(10)
    print ar_tags
    
    while not rospy.is_shutdown():
        #look up transforms if available for ar tags listed
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
        except:
            continue
        
        # YOUR CODE HERE
        #  The code should compute the twist given 
        #  the translation and rotation between arZ and ar1
        #  Then send it publish it to the zumy
        rbt = return_rbt(trans=trans, rot=rot)
        twist = compute_twist(rbt=rbt)
        cmd = Twist()
        cmd.linear.x = twist[0][0]/5
        cmd.linear.y = twist[0][1]/5
        cmd.linear.z = twist[0][2]/5
        cmd.angular.x = twist[1][0]
        cmd.angular.y = twist[1][1]
        cmd.angular.z = twist[1][2]
        zumy_vel.publish(cmd)
        rospy.sleep(0.1)
  
if __name__=='__main__':

    rospy.init_node('follow_ar_tag_twist')
    if len(sys.argv) < 4:
        print('Use: follow_ar_tag_manual.py [ zumy name ] [ AR tag number for goal] [ AR tag number for Zumy] ')
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]
    print("ASDF")

    #callback function
    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    rospy.spin()
