#!/usr/bin/env python

import rospy
from follow.srv import Scan

table_center = [0.6, .4]

def test_scan_server():
	rospy.init_node('test_scan_server', anonymous=True)
	rospy.wait_for_service('scan_server')
	perform_scan = rospy.ServiceProxy('scan_server', Scan, persistent=True)
	# test_request = Scan(table_center)
	print 'Performing scan...'
	response = perform_scan(table_center)
	tag_numbers, tag_poses = response.tagNumbers, response.arTagPoses
	print 'Done with scan.'
	print tag_numbers
	print response.arTagPoses


if __name__ == '__main__':
	test_scan_server()
