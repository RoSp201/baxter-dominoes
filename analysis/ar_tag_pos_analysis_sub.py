#!/usr/bin/env python
import rospy
from defaultdict import defaultdict
import matplotlib.pyplot as plt
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarker


waiting_dict = defaultdict(list)
pos_dict = defaultdict(dict)


def desired_position_sub(msg):
    global waiting_dict
    tag_num = msg.id
    tag_pos = msg.pose.pose.position
    waiting_dict[tag_num].append(tag_pos)


def actual_position_sub(msg):
    global waiting_dict
    global pos_dict
    tag_num = msg.id
    tag_pos = msg.pose.pose.position
    if tag_num in waiting_dict and len(waiting_dict[tag_num]) > 0:
        des_pos = waiting_dict[tag_num].pop(0)
        pos_dict[tag_num][des_pos] = actual_pos


def analyze_data():
    des_actual_pos = [des_actual for ar_tag_pair_dict in pos_dict.values() for des_actual in ar_tag_pair_dict.items()]
    des_actual_pos = [(np.array([des.x, des.y]), np.array([actual.x, actual.y])) for (des, actual) in des_actual_pos]
    xy_errors = np.array([des-actual for (des, actual) in des_actual_pos])
    abs_errors = np.linalg.norm(xy_errors, axis=1)
    plt.figure()
    plt.plot(xy_errors)
    plt.title('Relative error from desired position (0, 0)')
    plt.show()
    plt.figure()
    plt.hist(abs_errors)
    plt.title('Norm of error')


def listener():

    rospy.init_node("ar_tag_pos_analysis", anonymous=True)
    rospy.Subscriber(GOAL_POSITION_TOPIC, AlvarMarker, desired_position_sub)
    rospy.Subscriber(ACTUAL_POSITION_TOPIC, AlvarMarker, actual_position_sub)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
    analyze_data()
