#!/usr/bin/env python

import rospy

from follow.srv import Scan


table_origin = [.2, -0.25]


def test_scan_server():
	rospy.wait_for_service('scan_server')
	perform_scan = rospy.ServiceProxy('scan_server', Scan, persistent=True)
	rospy.init_node('test_scan_server', anonymous=True)
	test_request = Scan(table_origin)
	perform_scan(test_request)


if __name__ == '__main__':
	test_scan_server()
