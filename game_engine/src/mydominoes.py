#!/usr/bin/env python2

import sys
import copy
import rospy
from geometry_msgs import PoseStamped, Pose, Point, Quaternion
#For the head nod:
import baxter_interface

#Our services:
from PickNPlace.srv import *
from Translate.srv import *
from DominoCordSrv.srv import *
from ImageSrv.srv import *
from follow.srv import *

"""Currently, this is an OOP skeleton that we should put the ROS in.
CTRL+F your name in all caps for anything I asked you to do.
"""

# Constants to figure out
VERT_VERT_OFFSET = .13
HORIZ_VER_OFFSET = 0xdaddb0dd
HAND_SPACE_OFFSET = .03
HAND_AR_NUM = 0
NUM_PLAYERS = 2

class Player:
    ar_map = {15: (6,2), \
            8:(4,1), \
            19:(6,5), \
            5:(3,1), \
            10:(5,1), \
            20:(6,4), \
            17:(6,3)}

    def __init__(self, hand, root):
        self.hand = hand # array of domino objects
        self.root = root # Domino representing the spinner
        self.turns_taken = 0
        self.seen = {} # Domino objects for dominos we've seen
        self.game_init()

    def game_init(self):
        #Find hand AR tag and populate both seen and hand with the dominoes in our hand.
        self.scan_for_dominoes()
        self.hand_coords = self.seen[HAND_AR_NUM]
        # Take away the root hand tag when we're constructing the list
        del self.seen[HAND_AR_NUM]
        self.hand = self.seen.values()
        self.seen[HAND_AR_NUM] = self.hand_coords
        self.root = self.get_next_domino()
        self.game_loop()

    def pick_from_boneyard(self):
        """Wait for a player to give us a domino"""
        # Make baxter signal that he can't make a move by nodding.
        baxter_interface.Head().command_nod()
        rospy.sleep(5)
        domino = None
        while not domino:
            domino = self.get_next_domino()
            sleep(5)
        pos_in_hand  = self.add_domino_to_hand((domino))
        hand_coords = copy.deepcopy(self.hand_base)
        # This assumes that Baxter's hand starts in the closest-left corner of the table and it grows to the right.
        hand_coords.position.y += (pos_in_hand + 1) * HAND_SPACE_OFFSET
        self.move_domino(domino, hand_coords)

    def add_domino_to_hand(self, domino):
        i = 0
        while i < len(self.hand) and self.hand[i]:
            i += 1
        self.hand.insert(i, domino)

    def game_loop(self):
        game_ended = False
        self.game_init()
        while not game_ended:
            while not self.turns_taken % NUM_PLAYERS == 0:
                rospy.sleep(5)
                newdoms = self.scan_for_dominoes()
                ## Make a move:
                self.turns_taken += len(newdoms)
            if not self.turns_taken % NUM_PLAYERS:
                self.take_turn()

    def take_turn(self):
        """Analyze the previously sensed board state and make the best greedy move if it exists.
        If it doesn't, pick from the boneyard but nodding to the players and waiting for a new tile
        to move into the hand area."""
        # Compute the move to get:
        open_spots = self.root.get_open_spots([])
        assert(open_spots)
        move = self.best_move_greedy(self.hand, self.open_spots)
        if move[0] == -1:
            # If we can't find a move, we draw.
            self.pick_from_boneyard()
            self.turns_taken += 1
            return
        domino_to_move = self.hand[move[0]]
        domino_to_move_to = open_spots[move[1]]
        # Calculate which direction to rotate the domino.
        LR = self.get_domino_direction(domino_to_move_to)
        rot  = None
        # If the domino with the open spot has its X axis pointing left:
        if LR == "L":
            if move[3] == "bottom" and move[4] == "top" or\
                    move[3] == "top" and move[4] == "bottom":
                rot = "L"
            else:
                rot = "R"
        # If the domino with the open spot has its X axis pointing right:
        if LR == "R":
            if move[3] == "bottom" and move[4] == "top" or\
                    move[3] == "top" and move[4] == "bottom":
                rot = "R"
            else:
                rot = "L"

        # Get the coordinates to move to and translate them into the world frame.
        move_to = domino_to_move_to.get_location_from_domino(move[3])
        rospy.wait_for_service("pose_translate_server")
        trans_move_to = PoseStamped()
        print "try to transform coordinates"
        try:
            translate = rospy.ServiceProxy("pose_translate_server", Translate)
            trans_move_to = translate(move_to, 'left_hand_camera_axis')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        print "coordinates successfully transformed."

        self.move_domino(domino_to_move, trans_move_to, rot)

    def get_domino_direction(self, domino):
        pose = domino.pose
        quats = pose.orientation
        if quats.x > 0:
            return "L"
        else:
            return "R"

    def move_domino(self, domino_to_move, move_to, rot=0):
        #move_to must be a posed stamp object
        #move_to = unmade_translate_coords(move_to)
        #get domino transformed coordinates
        #domino to move.location needs to be a poseStamped object
        #third arg of pick_n_place is the direction that baxter needs to rotate the domino on the board.

        # place the domino on the board.
        print "try to place domino"
        rospy.wait_for_service("pick_n_place_server")
        try:
            pick_n_place = rospy.ServiceProxy("pick_n_place_server", PickNPlace)
            response = pick_n_place(domino_to_move.location, move_to, rot)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        print "domino placed successfully."

        # We don't have to add the domino to seen because we've already seen it in our hand.
        self.turns_taken += 1

    def scan_for_dominoes(self, num_dominoes = 0):
        """Finds all new dominoes currently detectable and enforces that we see no more than num_dominoes new ones.
        A value of zero indicates that any number of new dominoes is allowed."""
        while 1:
        ## Find new dominoes:
        dominoes = self.ROSSRV_SCAN()
        newdoms = []
        for domino in dominoes:
            # Convert the domino pips to our notation:
            if not domino in list(self.seen):
                orientation = domino[3]
                orientation_obj = convert_orientation_to_object #This is pseudocoded out because we don't know the specifics of the scan yet.
                pose_obj = get_pose_from_domino #this is pseudocoded out because we don't know the specifics of the CV return values yet.
                new_domino = Domino(pose_obj, orientation_obj)
                newdoms.append(domino)
                self.seen.add(domino)
        # Make sure that we're not too off the mark on how many new dominoes we should have.
        if num_dominoes and len(newdoms) > num_dominoes:
            raise Exception
        return newdoms

    def get_next_domino(self):
        return self.scan_for_dominoes(1)[0]

    def ROSSRV_SCAN(self):
        # TODO: call the new scan service, and put the returned poses into domino objects.
        pass

    def best_move_greedy(self, hand, open_spots):
        hiscore = 0
        #Position in hand, position in open_spots, side of the moving domino, side of the stationary domino
        best_play = (-1, -1, "ERR", "ERR")
        for spot in range(len(open_spots)):
            for i in range(len(hand)):
                curr = hand[i]
                currscore= score(curr)
                if currscore < hiscore:
                    continue
                if curr[0] == open_spots[spot][0]:
                    hiscore = currscore
                    best_play = (i, spot, "top", "top")
                elif curr[0] == open_spots[spot][1]:
                    hiscore = currscore
                    best_play = (i, spot, "top", "bottom")
                elif curr[1] == open_spots[spot][0]:
                    hiscore = currscore
                    best_play = (i, spot, "bottom", "top")
                elif curr[1] == open_spots[spot][1]:
                    hiscore = currscore
                    best_play = (i, spot, "bottom", "bottom")
        return best_play


# Class for keeping board state:
class Domino:
    def __init__(self, pips, pose):
        # The syntax for pips is (top pip, bottom pip)
        self.pips = pips
        self.sides = {"top": None, "bottom": None, "left": None, "right": None}
        self.pose = pose

    def __eq__(self, other):
        if self.pips == other.pips:
            return True
        return False

    def place(self, otherdom, pos):
        """ Places otherdom at this domino's pos"""
        assert not self.sides[pos]
        self.sides[pos] = otherdom
        otherdom.sides["bottom"] = self

    def get_open_spots(self, seen):
        """Currently, we're operating on a simplified domino model wherein dominoes can only be in a line.
        This just translates to finding the ends of a linked list."""
        top = self.sides["top"]
        bottom = self.sides["bottom"]
        if top and not top in seen:
            spots.append(top.get_open_spots()[0])
        else:
    def find_domino(self, pips):
        if self.pips == pips:
            return self
        else:

    def get_location_to_move_to(self, side):
        """Returns the offset from this domino's origin to apply to a domino if placing it at side side, specified in this domino's reference frame."""
        if "side" == "top":
            return (VERT_VERT_OFFSET, 0 0)
        if "side" == "bottom":
            return (-VERT_VERT_OFFSET, 0, 0)
        if "side" == "left":
            return (0, HORIZ_VERT_OFFSET, 0)
        if "side" == "right":
            return (0, -HORIZ_VERT_OFFSET, 0)
            spots.append(self.pips[0])
        if bottom and not bottom in seen:
            spots.append(bottom.get_open_spots()[1])
        else:
            spots.append(self.pips[1])
        return spots

    def score(self):
        """Returns the sum of the pips on a domino.
        We want to get rid of dominoes with higher scores."""
        return sum(self.pips)

    def find_domino(self, pips):
        if self.pips == pips:
            return self
        else:

    def get_location_to_move_to(self, side):
        """Returns the offset from this domino's origin to apply to a domino if placing it at side side, specified in this domino's reference frame."""
        temp = Pose()
        if "side" == "top":
            temp.position.x += VERT_VERT_OFFSET
        if "side" == "bottom":
            temp.position.x -= VERT_VERT_OFFSET
        if "side" == "left":
            temp.position.y += VERT_VERT_OFFSET
        if "side" == "right":
            temp.position.y -= VERT_VERT_OFFSET
