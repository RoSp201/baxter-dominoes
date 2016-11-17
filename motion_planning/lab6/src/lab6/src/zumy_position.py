#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist, TransformStamped, Quaternion

from ar_tag_subs import compute_twist, return_rbt
from kalman_zumy.srv import NuSrv


listener = None

def follow_ar_tag(zumy, ar_tags):

    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher('%s/cmd_vel' % zumy, Twist, queue_size=2)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            #(trans, rot) = listener.lookupTransform(ar_tags['arZ'])
            tf = listener.lookupTransform(ar_tags['arZ'])
            trans, rot = tf
        except:
            continue
        
        # YOUR CODE HERE
        #  The code should compute the twist given 
        #  the translation and rotation between arZ and ar1
        #  Then send it publish it to the zumy
        update = rospy.ServiceProxy('innovation', NuSrv)
        msg = TFMessage()
        linear = Vector3()
        linear.x = trans[0]
        linear.y = trans[1]
        linear.z = trans[2]
        rotational = Quaternion()
        rotational.x = trans[0]
        rotational.y = trans[1]
        rotational.z = trans[2]
        rotational.z = trans[3]
        msg.transform.linear = linear
        msg.transform.rotation = rotational
        msg.origin_tag = "arZ"
        update(msg)
  
if __name__=='__main__':
    rospy.init_node('follow_ar_tag_twist')
    if len(sys.argv) < 4:
        print('Use: follow_ar_tag_manual.py [ zumy name ] [ AR tag number for goal] [ AR tag number for Zumy] ')
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    rospy.spin()
