#!/usr/bin/env python

import rospy

from follow.srv import Scan


table_center = [0.75, 0]


def test_scan_server():
	rospy.init_node('test_scan_server', anonymous=True)
	rospy.wait_for_service('scan_server')
	perform_scan = rospy.ServiceProxy('scan_server', Scan, persistent=True)
	# test_request = Scan(table_center)
	perform_scan(table_center)


if __name__ == '__main__':
	test_scan_server()
