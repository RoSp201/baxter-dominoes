#!/usr/bin/env python
from collections import defaultdict
import sys
from threading import Condition
try:
    from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
    import rospy
    from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
    from ar_track_alvar_msgs.msg import AlvarMarkers
    from follow.srv import *
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

import numpy as np


# Continuously populated dictionary of seen AR tags
# raw_tags contains each tag seen in the last tag reading
# If a tag is kept from one read to the next, adds list of AlvarMarker for that tag
raw_tags = defaultdict(list) # tag number: [(confidence, Position, Orientation), ...]

scan_params = dict()

FIRST_REQUIRED_COUNT = 2
FIRST_Z = 0.1
first_tags = dict() # tag number: Pose
FIRST_FOV = np.array([0.1, 0.1])
# Once a tag in raw_tags has a count of FIRST_REQUIRED_COUNT, it's added to first_tags
scan_params['FIRST_SCAN'] = {
    'REQUIRED_COUNT': FIRST_REQUIRED_COUNT,
    'TAGS': first_tags,
    'FOV': FIRST_FOV,
    'Z': FIRST_Z
}

# After first scan, go back to tags and rescan with higher accuracy
LAST_REQUIRED_COUNT = 5
LAST_Z = 0.1
last_tags = dict()
LAST_FOV = np.array([0.1, 0.1])
scan_params['LAST_SCAN'] = {
    'REQUIRED_COUNT': LAST_REQUIRED_COUNT,
    'TAGS': last_tags,
    'FOV': LAST_FOV,
    'Z': LAST_Z
}

cur_params = scan_params['FIRST_SCAN']

# MoveGroupCommander arm object
left_arm = None

velocity_scale_factor = 0.1
eef_step = 0.01

# Counts scans completed at any one location
scan_counter = 0
# Synchronization for scanning and moving
scan_cv = Condition()
scan_call_in_progress = False


def hold_scan():
    # Enters with cv acquired
    global scan_counter, scan_call_in_progress
    raw_tags.clear()
    scan_counter = 0
    scan_call_in_progress = True
    while ((scan_counter < cur_params['REQUIRED_COUNT']) or
           (raw_tags)):
        # Release lock and block
        scan_cv.wait()
        # Wait returns with lock acquired
    # Exits with cv acquired
    print(cur_params['TAGS'].keys())
    print('Finished one location')
    scan_call_in_progress = False


def grid_table(table_center, table_size):
    # Number of lengthwise scans of table
    n_scans = np.ceil(table_size/cur_params['FOV']).astype(int)
    nx, ny = n_scans
    scan_spacing = table_size/n_scans # Space between lengthwise scans
    dx, dy = scan_spacing
    print(n_scans)
    print(scan_spacing)
    front_right_corner = table_center - (table_size/2)
    next_xy = front_right_corner + (scan_spacing/2)
    points = np.zeros((nx*ny, 2))
    points[0, :] = next_xy
    direction = 1
    for ix_scan in range(nx):
        # Only move to next scan line after first scan
        if ix_scan != 0:
            # First move away from Baxter to start of next horizontal scan line
            next_xy[0] += dx
            points[ix_scan*ny, :] = next_xy
        # Then move horizontally across the table
        for iy_scan in range(1, ny):
            next_xy[1] += direction*dy
            points[ix_scan*ny+iy_scan, :] = next_xy
        direction *= -1
    return points


def scan_points(scan_xy):
    next_xy = scan_xy[0, :]
    next_point = Point(next_xy[0], next_xy[1], cur_params['Z'])
    next_quat = Quaternion(0, -1, 0, 0)
    next_pose = Pose(next_point, next_quat)

    for move_count in range(scan_xy.shape[0]):
        next_xy = scan_xy[move_count, :]
        next_pose.position.x, next_pose.position.y = next_xy
        move_to_position(next_pose)
        hold_scan()


def handle_scan(request):
    """
    Scan.srv format:

    # Request to scan table for Dominoes
    # x0,y0 coordinates of front right corner of table
    # Note that left is positve y and forward is positive x
    #          x
    #    ______^______
    #   |      :      |
    # y<|......O......|
    #   |______:______|._(x0,y0)
    #
    uint32[] tableCenter
    ---
    uint32[] tagNumbers
    geometry_msgs/Pose[] arTagPoses
    """
    global cur_params
    print('Scanning from service...')
    scan_cv.acquire()

    for val in scan_params.values():
        val['TAGS'].clear()

    cur_params = scan_params['FIRST_SCAN']
    table_center, table_size = np.array(request.tableCenter), np.array(request.tableSize)
    scan_xy = grid_table(table_center, table_size)
    scan_points(scan_xy)
    print('FIRST SCAN FOUND {}'.format(first_tags.keys()))

    cur_params = scan_params['LAST_SCAN']
    scan_xy = np.array([(pose.position.x, pose.position.y)
                        for pose in first_tags.values()])
    scan_points(scan_xy)
    print('LAST SCAN FOUND {}'.format(last_tags.keys()))

    scan_cv.release()
    # Converts last_tags dictionary to [(1, 2, ...), (Pose1, Pose2, ...)]
    return zip(*last_tags.items())


def ar_tag_filter(msg):
    """
    AlvarMarkers msg format:

    std_msgs/Header header
    ar_track_alvar_msgs/AlvarMarker[] markers

    AlvarMarker format:
    std_msgs/Header header
    uint32 id
    uint32 confidence
    geometry_msgs/PoseStamped pose
    """
    # Only scan when scanner is calling hold_scan()
    global scan_counter
    if (not scan_call_in_progress) or (not scan_cv.acquire(blocking=False)):
        return

    # Extract useful information
    current_tags = [(marker.id, marker.pose) for marker in msg.markers
                    if marker.id not in cur_params['TAGS'] # Exclude tags that already passed the filter
                    and marker.id < 32] # Accept AR tags 0-31
    if current_tags:
        current_tag_ids = set(zip(*current_tags)[0])
        # Remove tags from raw dict that aren't seen this round
        for old_tag_id in raw_tags.keys():
            if old_tag_id not in current_tag_ids:
                del raw_tags[old_tag_id]
        # Add all poses seen
        for (tag_id, pose) in current_tags:
            print('Found tag {}'.format(tag_id))
            if not all(np.abs(np.array([pose.pose.position.x, pose.pose.position.y])) <  cur_params['FOV']/2):
                print('Tag {} is outside FOV'.format(tag_id))
                continue

            rospy.wait_for_service('translate_server')
            print('try to transform coordinates')
            try:
                translate_server = rospy.ServiceProxy('translate_server', Translate)
                pose = translate_server(pose, 'base').output_pose_stamped
            except rospy.ServiceException as e:
                print('Service call failed: %s' % e)
            print('coordinates successfully transformed.')
            print('Pose position: {} {}'.format(pose.pose.position.x, pose.pose.position.y))
            tag_list = raw_tags[tag_id]
            tag_list.append((pose.pose.position, pose.pose.orientation))
            # If pose seen enough times, average pose information and add to tag dict
            if len(tag_list) == cur_params['REQUIRED_COUNT']:
                print('Moving tag {} to filtered list.'.format(tag_id))
                positions, orientations = zip(*tag_list)
                avg_pos = np.mean(
                    np.array([(pos.x, pos.y, pos.z) for pos in positions]),
                    axis=0
                )
                filt_point = Point(*avg_pos)
                avg_quat = np.mean(
                    np.array([(quat.x, quat.y, quat.z, quat.w) for quat in orientations]),
                    axis=0
                )
                filt_quat = Quaternion(*avg_quat)
                cur_params['TAGS'][tag_id] = Pose(filt_point, filt_quat)
                del raw_tags[tag_id]
    scan_counter += 1
    print('Scan counter: {}'.format(scan_counter))
    scan_cv.notify()
    scan_cv.release()


def move_to_position(goal_pose):
    plan, _ = left_arm.compute_cartesian_path(
        [left_arm.set_current_state_to_start_state(), goal_pose],     # waypoints to follow with end
        eef_step,   # eef_step
        0.0         # jump_threshold
    )
    left_arm.execute(plan)
    rospy.sleep(1.0) #add this to see if baxter's action trajectory server will stop complaining about reaching max velocity threshold during scan.


def init_motion():
    global left_arm
    roscpp_initialize(sys.argv)
    left_arm = MoveGroupCommander('left_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(7.0)
    left_arm.allow_replanning(True)
    left_arm.set_end_effector_link('left_gripper')
    left_arm.set_pose_reference_frame('base')
    left_arm._g.set_max_velocity_scaling_factor(velocity_scale_factor)

def init_filter():
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_tag_filter)

def scan_server():
    rospy.init_node('scan_server')
    init_filter()
    init_motion()
    rospy.Service('scan_server', Scan, handle_scan)
    print('\nScan server ready!\n\n')
    rospy.spin()

def test():
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    table_center = np.array([0.6, 0.4])
    table_size = np.array([0.5, 0.8])
    gridded_points = grid_table(table_center, table_size)
    table = Rectangle((table_center-table_size/2),
                      table_size[0], table_size[1],
                      fill=None)
    plt.figure()
    ax = plt.gca()
    ax.add_patch(table)
    plt.scatter(gridded_points[:, 0], gridded_points[:, 1])
    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
    if not ROS_AVAILABLE or (len(sys.argv) > 1 and sys.argv[1] == '-t'):
        test()
    elif ROS_AVAILABLE:
        scan_server()
    else:
        print('ROS not available or test flag not set')
