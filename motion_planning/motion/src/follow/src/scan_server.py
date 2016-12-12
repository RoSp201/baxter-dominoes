#!/usr/bin/env python
from collections import defaultdict
from threading import Condition
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from follow.srv import *


# Table dimensions in meters
# TODO: use the first dim when not debugging
#dim = np.array([0.5, 0.75])
dim = np.array([0.25, .5])
# x and y FOV
fov = np.array([0.15, 0.15])
# Number of lengthwise scans of table
n_scans = np.ceil(dim/fov).astype(int)
nx, ny = n_scans
# Space between lengthwise scans
scan_spacing = dim/n_scans
dx, dy = scan_spacing
print n_scans
print scan_spacing

# Constant scan height above the table
z0 = 0.10

# MoveGroupCommander arm object
left_arm = None

# translate_service proxy
translate_server = None


# Continuously populated dictionary of seen AR tags
# raw_tags contains each tag seen in the last tag reading
# If a tag is kept from one read to the next, adds list of AlvarMarker for that tag
raw_tags = defaultdict(list) # tag number: [(confidence, Position, Orientation), ...]
# Once a tag in raw_tags has a count of REQUIRED_COUNT, it's added to seen_tags
REQUIRED_COUNT = 2
MAX_SCANS = REQUIRED_COUNT*2
seen_tags = dict() # tag number: Pose
# Counts scans completed at any one location
scan_counter = 0
# Synchronization for scanning and moving
scan_cv = Condition()
scan_call_in_progress = False


def hold_scan():
    # Enters with cv acquired
    global scan_counter, scan_call_in_progress
    scan_counter = 0
    scan_call_in_progress = True
    while scan_counter < MAX_SCANS:
        # Release lock and block
        scan_cv.wait()
        # Wait returns with lock acquired
    # Exits with cv acquired
    print seen_tags.keys()
    print 'Finished one location'
    scan_call_in_progress = False


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
    print('Scanning from service...')
    scan_cv.acquire()
    origin = np.array(request.tableCenter)
    x0, y0 = origin - (dim/2)

    # Move Baxter's camera to the front right corner of the table
    next_point = Point(x0, y0, z0)
    next_quat = Quaternion(0, -1, 0, 0)
    next_pose = PoseStamped()
    next_pose.pose.position = next_point
    next_pose.pose.orientation = next_quat
    move_to_position(next_pose)

    raw_tags.clear()
    seen_tags.clear()

    direction = 1
    for ix_scan in range(nx):
        # Only move to next scan line after first scan
        if ix_scan != 0:
            # First move away from Baxter to start of next horizontal scan line
            next_pose.pose.position.x += dx
            move_to_position(next_pose)
        hold_scan()

        # Then move horizontally across the table
        for iy_scan in range(ny-1):
            next_pose.pose.position.y += direction*dy
            move_to_position(next_pose)
            hold_scan()

        direction *= -1

    next_pose.pose.position.x, next_pose.pose.position.y = origin
    move_to_position(next_pose)

    scan_cv.release()
    # Converts seen_tags dictionary to [(1, 2, ...), (Pose1, Pose2, ...)]
    return zip(*seen_tags.items())


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
    if ((MAX_SCANS <= scan_counter) or (not scan_call_in_progress)
        or (not scan_cv.acquire(blocking=False))):
        return

    # Extract useful information, and exclude tags that already passed the filter (only accept ar tags 0-31)
    current_tags = [(marker.id, marker.pose) for marker in msg.markers if marker.id not in seen_tags and marker.id < 32]
    if current_tags:
        current_tag_ids = set(zip(*current_tags)[0])
        # Remove tags from raw dict that aren't seen this round
        for old_tag_id in raw_tags.keys():
            if old_tag_id not in current_tag_ids:
                del raw_tags[old_tag_id]
        # Add all poses seen
        for (tag_id, pose) in current_tags:
            if not all(np.abs(np.array([pose.pose.position.x, pose.pose.position.y])) <  fov/2):
                continue

            rospy.wait_for_service("translate_server")
            print "try to transform coordinates"
            try:
                translate_server = rospy.ServiceProxy("translate_server", Translate)
                pose = translate_server(pose, 'base').output_pose_stamped
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            print "coordinates successfully transformed."
            print "Pose position: \n{}".format(pose.pose.position)
            tag_list = raw_tags[tag_id]
            tag_list.append((pose.pose.position, pose.pose.orientation))
            # If pose seen enough times, average pose information and add to seen_tags
            if len(tag_list) == REQUIRED_COUNT:
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
                seen_tags[tag_id] = Pose(filt_point, filt_quat)
                del raw_tags[tag_id]
    scan_counter += 1
    print 'Scan counter:',scan_counter
    scan_cv.notify()
    scan_cv.release()


def move_to_position(goal_pose):
    plan, _ = left_arm.compute_cartesian_path(
        [goal_pose.pose],    # waypoints to follow with end
        0.01,           # eef_step
        0.0             # jump_threshold
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

def init_filter():
    global translate_server
    rospy.wait_for_service('translate_server')
    translate_server = rospy.ServiceProxy('translate_server', Translate, persistent=True)
    #rospy.init_node('ar_tag_filter', anonymous=True)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_tag_filter)

def scan_server():
    rospy.init_node('scan_server')
    init_filter()
    init_motion()
    rospy.Service('scan_server', Scan, handle_scan)
    print('\nScan server ready!\n\n')
    rospy.spin()


if __name__ == '__main__':
    scan_server()
