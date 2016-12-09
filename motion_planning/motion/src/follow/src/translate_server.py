import rospy
import tf
from tf2_msgs.msg import TFMessage
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import ar_tag_pos as arp
import numpy as np
#from ../srv import Translate.srv #so service knows format of message type for callback

def handle_translate(coords):
    #this function will turn camera coordinates into base frame coordinates 
    #we can actually make the gripper orientation in terms of the ar tag frame orientation as seen by the base frame
    
    #input will be a poseStamped() object referenced in the camera frame

    #make sure two lines below are uncommented in package.xml
    #also add the same into CMakeList.txt
    #<build_depend>message_generation</build_depend>
  	#<run_depend>message_runtime</run_depend>

    x = coords.pose_stamped.pose.position.x
    y = coords.pose_stamped.pose.position.y
    z = coords.pose_stamped.pose.position.z
    input_coords = np.array([x,y,z,1])

    tf_listener = tf.TransformListener()
    found = False
    while not found:

    	try:
            #transform frame coords, so with respect to base frame 
            #note, can use ar tag frame, so that gripper will always be oriented correctly with the ar tag it is picking up
            tf_listener.waitForTransform("base", "left_hand_camera_axis", rospy.Time(0), rospy.Duration(4.0))
            transform = tf_listener.lookupTransform("base", "left_hand_camera_axis", rospy.Time(0))
            (trans, rot) = transform

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print "Cam coordinates: ", x, y, z
        found = True
        rbt = arp.return_rbt(trans=trans, rot=rot)
        #transforms the entire pose, not just position
        base_gripper_pose = tf2_geometry_msgs.do_transform_pose(coords.pose_stamped, transform)

        #original method if above function doesn't work properly
        base_coords = rbt.dot(coords_cam_frame)
        base_coords = base_coords.reshape(4,1)
        x2 = base_coords.item(0)
        y2 = base_coords.item(1)
        z2 = base_coords.item(2)
        print "Base coordinates: ", x2, y2, z2
        
        output_pose_stamped = PoseStamped()
        output_pose_stamped.header.frame_id = "base"
        output_pose_stamped.pose.position.x = x2
        output_pose_stamped.pose.position.y = y2
        output_pose_stamped.pose.position.z = z2

        #if want new method, just uncomment line below and switch out:

        #return base_gripper_pose

        return return output_pose 


def translate_server():
    rospy.init_node("pose_translate_server")
    s = rospy.Service("pose_translate_server", Translate, handle_translate)
    print "Coordinate translator server ready to use!" 
    rospy.spin()

if __name__ == "__main__":
    translate_server()

