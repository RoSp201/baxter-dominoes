#!/usr/bin/env python
# import rospy
# import tf
# import tf2_ros
# from tf2_msgs.msg import TFMessage
# import tf2_geometry_msgs
# from geometry_msgs.msg import PoseStamped, Pose
# import ar_tag_pos as arp
from collections import defaultdict
from threading import Condition
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from follow.srv import Scan


# Table dimensions in meters
dim = np.array([0.5, 1])
# x and y FOV
fov = np.array([.2, .2])
# Number of lengthwise scans of table
n_scans = np.ceil(dim/fov)
nx, ny = n_scans
# Space between lengthwise scans
scan_spacing = dim/n_scans
dx, dy = scan_spacing

# Constant scan height above the table
z0 = .2

# MoveGroupCommander arm object
left_arm = None

# translate_service proxy
translate_pose = None

# Number of attempts to hit goal position
move_attempts = 3


# Continuously populated dictionary of seen AR tags
# raw_tags contains each tag seen in the last tag reading
# If a tag is kept from one read to the next, adds list of AlvarMarker for that tag
raw_tags = defaultdict(list) # tag number: [(confidence, Position, Orientation), ...]
# Once a tag in raw_tags has a count of REQUIRED_COUNT, it's added to seen_tags
REQUIRED_COUNT = 3
seen_tags = dict() # tag number: Pose
# Counts scans completed at any one location
scan_counter = 0
# Synchronization for scanning and moving
scan_cv = Lock()


def hold_scan():
    # Enters with cv acquired
    global scan_counter
    scan_counter = 0
    while scan_counter < REQUIRED_COUNT*2:
        # Release lock and block
        scan_cv.wait()
        # Wait returns with lock acquired
    # Exits with cv acquired


def handle_scan(request):
    """
    Scan.srv format:

    # x0,y0 coordinates of front right corner of table
    # Note that left is positve y and forward is positive x
    #          x
    #    ______^
    #   |      |
    #   |______|
    # y<       ^=(x0, y0)
    uint32[] tableOrigin
    ---
    uint32[] tagNumbers
    geometry_msgs/Pose[] arTagPoses
    """
    x0, y0 = request.tableOrigin
    x1, y1 = dim + [x0, y0]

    # Move Baxter's camera to the front right corner of the table
    next_point = Point(x0, y0, z0)
    next_quat = Quaternion(0, -1, 0, 0)
    next_pose = Pose(next_point, next_quat)
    move_to_position(next_pose)

    seen_tags.clear()

    for ix_scan in range(nx):
        # Only move to next scan line after first scan
        if ix_scan != 0:
            # First move away from Baxter to start of next horizontal scan line
            next_pose.position.x += dx
            #####In case we actually need to create a new Pose for each move
            #### cur_pose = next_pose
            #### next_point = Point(curPose.position.x+dx, curPose.position.y, z0)
            #### next_quat = Quaternion(0, -1, 0, 0)
            #### next_pose = Pose(next_point, next_quat)
            move_to_position(next_pose)
        hold_scan()

        # Then move horizontally across the table
        for iy_scan in range(ny-1):
            next_pose.position.y += dy
            move_to_position(next_pose)
            hold_scan()

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
    if not scan_cv.acquire(blocking=false):
        return

    # Extract useful information, and exclude tags that already passed the filter
    current_tags = [(marker.id, marker.confidence, marker.pose) for marker in msg.markers if marker.id not in seen_tags]
    current_tag_ids = set(zip(*current_tags)[0])
    # Remove tags from raw dict that aren't seen this round
    for old_tag_id in raw_tags.keys():
        if old_tag_id not in current_tag_ids:
            del raw_tags[old_tag_id]
    # Add all poses seen
    for marker in current_tags:
        tag_id, confidence, pose = marker.id, marker.confidence, marker.pose
        pose = translate_server(pose, 'base').pose
        tag_list = raw_tags[tag_id]
        tag_list.append((confidence, pose.pose.position, pose.pose.Orientation))
        # If pose seen enough times, average pose information and add to seen_tags
        if len(tag_list) == REQUIRED_COUNT:
            confidences, positions, orientations = zip(*tag_list)
            total_confidence = sum(confidences)
            filt_point = Point(
                np.average([(pos.x, pos.y, pos.z) for pos in positions],
                           axis=0, weights=confidences)
            )
            filt_quat = Quaternion(
                np.average([(quat.x, quat.y, quat.z, quat.w) for quat in orientations],
                           axis=0, weights=confidences)
            )
            seen_tags[tag_id] = Pose(filt_point, filt_quat)
    global scan_counter
    scan_counter += 1
    scan_cv.release()


def move_to_position(goal_pose):
    for i in range(move_attempts):
        plan, fraction = left_arm.compute_cartesian_path(
            [goal_pose],    # waypoints to follow with end
            0.01,           # eef_step
            0.0             # jump_threshold
        )
        left_arm.execute(plan)
        rospy.sleep(5)
        if 0.9 < fraction:
            return


def init_motion():
    global left_arm
    roscpp_initialize(sys.argv)
    left_arm = MoveGroupCommander('left_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(5.0)
    left_arm.allow_replanning(True)
    left_arm.set_end_effector_link("left_gripper")
    left_arm.set_pose_reference_frame('base')

def init_filter():
    global translate_pose
    rospy.wait_for_service('translate_server')
    translate_pose = rospy.ServiceProxy('translate_server', Translate)
    rospy.init_node("ar_tag_filter", anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_tag_filter)

def scan_server():
    init_filter()
    init_motion()
    rospy.init_node("scan_server")
    rospy.Service("scan_server", Translate, handle_scan)
    print("\n\Scan server ready!\n\n")
    rospy.spin()


if __name__ == "__main__":
    scan_server()
