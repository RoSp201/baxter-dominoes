#!/usr/bin/env python2

import sys
import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# For the head nod:
import baxter_interface

# Our services:
from follow.srv import *

"""Currently, this is an OOP skeleton that we should put the ROS in.
CTRL+F your name in all caps for anything I asked you to do.
"""

# Constants to figure out
VERT_VERT_OFFSET = .13
HORIZ_VER_OFFSET = 0.18
HAND_SPACE_OFFSET = .10 #0.03
HAND_AR_NUM = 31
NUM_PLAYERS = 2
TAGS_TO_PIPS = {
                0: (1, 0),
                1: (2, 0),
                2: (2, 1),
                28: (3, 0),
                4: (4, 0),
                5: (3, 1),
                6: (3, 2),
                7: (5, 0),
                8: (4, 1),
                9: (6, 0),
                10: (5, 1),
                11: (4, 2),
                12: (5, 2),
                13: (4, 3),
                14: (6, 1),
                15: (6, 2),
                16: (5, 3),
                17: (6, 3),
                18: (5, 4),
                19: (6, 5),
                20: (6, 4),
                HAND_AR_NUM: (0, 0),
                }

TABLE_CENTER = [0.7, .4]
SCAN_SIZE = [.5, .5]


class Player:
    def __init__(self):
        self.turns_taken = 1
        self.seen = {}  # Domino objects for dominos we've seen
        self.game_init()

    def game_init(self):
        print "\n\n*** START OF GAME ***\n\n"
        print "Scanning for the hand"
        # Find hand AR tag and populate both seen and hand with the dominoes in our hand.
        self.scan_for_dominoes(TABLE_CENTER)
        self.hand_coords = self.seen[HAND_AR_NUM].pose_st
        temp = self.seen[HAND_AR_NUM]
        # Take away the root hand tag when we're constructing the list
        del self.seen[HAND_AR_NUM]
        self.hand = self.seen.values()
        i = 1
        print "SEEN: {}".format(self.seen)
        for domino in self.hand:
            hand_coords = copy.deepcopy(self.hand_coords)
            hand_coords.pose.position.y -= (i * HAND_SPACE_OFFSET)
            i += 1
            # move_domino takes a domino and a pose stamped.
            self.move_domino(domino, hand_coords)
        self.seen[HAND_AR_NUM] = temp
        print "\nNow scanning for the spinner..."
        self.spinner = self.get_next_domino()
        print "\nSpinner was recorded. Now starting main game loop..."
        self.game_loop()

    def pick_from_boneyard(self):
        """Wait for a player to give us a domino"""
        # Make baxter signal that he can't make a move by nodding. #NOTE: Make sure head_action server running
        print "\nBaxter nodded in request for a new domino."
        rospy.sleep(5.0)
        domino = None
        while not domino:
            domino = self.get_next_domino()
            rospy.sleep(2.0)
        pos_in_hand = self.add_domino_to_hand(domino)
        hand_coords = copy.deepcopy(self.hand_coords)
        # This assumes that Baxter's hand starts in the closest-left corner of the table and it grows to the right.
        hand_coords.pose.position.y -= ((pos_in_hand + 1) * HAND_SPACE_OFFSET)
        self.move_domino(domino, hand_coords)

    def add_domino_to_hand(self, domino):
        i = 0
        while i < len(self.hand) and self.hand[i]:
            i += 1
        if i == len(self.hand):
            i += 1
        self.hand.insert(i, domino)
        return i

    def game_loop(self):
        game_ended = False
        while not game_ended:
            #while it's baxter's turn
            while self.turns_taken % NUM_PLAYERS == 0:
                rospy.sleep(2)
                newdoms = []
                spots = self.spinner.get_open_spots([])
                print("Scanning through open spots: {}".format([(spot[0].pips, spot[1]) for spot in spots]))
                for spot in spots:
                    emptypos = spot[0].get_location_to_move_to(spot[1])
                    #pos = (spot[0].pose_st.pose.position.x, spot[0].pose_st.pose.position.y)
                    pos = (emptypos.pose.position.x, emptypos.pose.position.y)
                    print("Scanning through spot {}".format(pos))
                    newscan = self.scan_for_dominoes(pos)
                    if newscan:
                        newdoms.extend([scan for scan in newscan])
                print("NEWDOMS: {}".format(newdoms))
                for newdom in newdoms:
                    norm = lambda p1, p2: math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 + (p1.pose.position.y - p2.pose.position.y)**2)

                    spot = min(spots, key= lambda dom: norm(newdom.pose_st, dom[0].pose_st) if (newdom.pips[0] in dom[0].pips) or (newdom.pips[1] in dom[0].pips) else 9999999999)
                    if spot[0].sides[spot[1]]:
                        print "ERR: found a domino at a spot where a domino already exists."
                    spot[0].sides[spot[1]] = newdom
                    print("new domino {} at spot {} of domino {}".format(newdom.pips, spot[1], spot[0].pips))

                    # Find out which side of the new domino just got added to
                    pips = None
                    if spot[1] == "top":
                        pips = spot[0].pips[0]
                    elif spot[1] == "bottom":
                        pips = spot[0].pips[1]

                    # Update the new domino's adjacent dominoes.
                    print("Appending domino {} to {}".format(newdom.pips, spot[0].pips))
                    if newdom.pips[0] == pips:
                        newdom.sides["top"] = spot[0]
                    elif newdom.pips[1] == pips:
                        newdom.sides["bottom"] = spot[0]
                    else:
                        print "Invalid move detected."

                    # We have a new turn for each new domino we see.
                    self.turns_taken += 1

            if self.turns_taken % NUM_PLAYERS:
                print "\nBaxter Taking turn..."
                self.take_turn()

    def take_turn(self):
        """Analyze the previously sensed board state and make the best greedy move if it exists.
        If it doesn't, pick from the boneyard by nodding to the players and waiting for a new tile
        to move into the hand area."""
        # Compute the move to get:
        open_spots = self.spinner.get_open_spots([])
        assert(open_spots)
        move = self.best_move_greedy(self.hand, open_spots)
        while move[0] == -1:
            # If we can't find a move, we draw.
            self.pick_from_boneyard()
            move = self.best_move_greedy(self.hand, open_spots)
        domino_to_move = self.hand[move[0]]
        print("DOMINO TO MOVE: {}".format(domino_to_move.pips))
        self.hand[move[0]] = None
        domino_to_move_to = open_spots[move[1]]
        print("DOMINO TO MOVE TO: {}".format(domino_to_move_to[0].pips))
        # Calculate which direction to rotate the domino.
        LR = domino_to_move_to[0].get_domino_direction()
        rot = ""
        # If the domino with the open spot has its X axis pointing left:
        if LR == "L":
            print("take_turn: Move to facing left")
            print(move[2])
            print(move[3])
            if move[2] == move[3]:
                print("setting rot to R")
                rot = "R"
            else:
                print("setting rot to L")
                rot = "L"

        # If the domino with the open spot has its X axis pointing right:
        if LR == "R":
            print("take_turn: Move to facing right")
            print(move[2])
            print(move[3])
            if move[2] == move[3]:
                print("setting rot to L")
                rot = "L"
            else:
                print("setting rot to R")
                rot = "R"

        move_to = domino_to_move_to[0].get_location_to_move_to(move[3])
        self.move_domino(domino_to_move, move_to, rot)
        domino_to_move.sides[move[2]] = domino_to_move_to[0]
        domino_to_move_to[0].sides[move[3]] = domino_to_move
        self.turns_taken += 1
        print "Take turn successful"
        #baxter_interface.Head().command_nod()
        #baxter_interface.Head().command_nod()

    def move_domino(self, domino_to_move, move_to, rot=""):
        print "Try to place domino {} at {} using pick_n_place...".format(domino_to_move, move_to)
        rospy.wait_for_service("pick_n_place_server")
        try:
            pick_n_place = rospy.ServiceProxy("pick_n_place_server", PickNPlace)
            response = pick_n_place(domino_to_move.pose_st, move_to, rot)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            domino_to_move.pose_st = move_to
            # Hard-code the orientation if we're rotating.
            if rot:
                domino_to_move.pose_st.pose.orientation.x = 0
                domino_to_move.pose_st.pose.orientation.y = 0
                if rot == "L":
                    domino_to_move.pose_st.pose.orientation.z = 1
                elif rot == "R":
                    domino_to_move.pose_st.pose.orientation.z = -1
                domino_to_move.pose_st.pose.orientation.w = 0
        print "Domino was placed successfully."

    def scan_for_dominoes(self, center, num_dominoes=0):
        """Finds all new dominoes currently detectable and enforces that we see no more than num_dominoes new ones.
        A value of zero indicates that any number of new dominoes is allowed."""
        while 1:
            ## Find new dominoes:
            dominoes = self.call_scan(center)
            newdoms = []
            for domino in dominoes:
                if domino not in list(self.seen):
                    print(domino.tag)
                    newdoms.append(domino)
                    self.seen[domino.tag] = domino
            # Make sure that we're not too off the mark on how many new dominoes we should have.
            if num_dominoes and len(newdoms) > num_dominoes:
                print "ERR: {} dominoes expected but {} dominoes found.".format(num_dominoes, len(newdoms))
            print("NEW DOMINOES: {}".format([(dom.pips, dom.pose_st) for dom in newdoms]))
            return newdoms

    def get_next_domino(self):
        dom = self.scan_for_dominoes(TABLE_CENTER, 1)
        if dom:
            return dom[0]
        else:
            print "No domino found. rescanning."
            return self.get_next_domino()

    def call_scan(self, center):
        tags, poses= self.blatnerize(center)
        dominoes = []
        for i in range(len(tags)):
            stamped = PoseStamped()
            stamped.header.frame_id = "base"
            stamped.pose = copy.deepcopy(poses[i])
            # This is to prevent bugs in translate_server.
            dominoes.append(Domino(TAGS_TO_PIPS[tags[i]], tags[i], stamped))
        return dominoes

    def blatnerize(self, center):
        print "\nTry to do a scan"
        tag_numbers, tag_poses = None, None
        move = False
        while not move:
            rospy.wait_for_service("scan_server")
            try:
                scan = rospy.ServiceProxy("scan_server", Scan, persistent=True)
                response = scan(center, SCAN_SIZE)
                tag_numbers, tag_poses = response.tagNumbers, response.arTagPoses
                move = True
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        print "Scan successful."
        return tag_numbers, tag_poses

    def best_move_greedy(self, hand, open_spots):
        hiscore = -1
        #Position in hand, position in open_spots, side of the moving domino, side of the stationary domino
        best_play = (-1, -1, "ERR", "ERR")
        print(open_spots)
        for spot in range(len(open_spots)):
            for i in range(len(hand)):
                curr = hand[i]
                if not curr:
                    continue
                currscore = curr.score(open_spots[spot][0])
                if currscore < hiscore:
                    continue
                if open_spots[spot][1] == "top":
                    if curr.pips[0] == open_spots[spot][0].pips[0]:
                        hiscore = currscore
                        best_play = (i, spot, "top", "top")
                    elif curr.pips[1] == open_spots[spot][0].pips[0]:
                        hiscore = currscore
                        best_play = (i, spot, "bottom", "top")

                if open_spots[spot][1] == "bottom":
                    if curr.pips[0] == open_spots[spot][0].pips[1]:
                        hiscore = currscore
                        best_play = (i, spot, "top", "bottom")
                    elif curr.pips[1] == open_spots[spot][0].pips[1]:
                        hiscore = currscore
                        best_play = (i, spot, "bottom", "bottom")

        return best_play


# Class for keeping board state:
class Domino:
    def __init__(self, pips, tag, pose_st):
        # The syntax for pips is (top pip, bottom pip)
        self.pips = pips
        self.sides = {"top": None, "bottom": None, "left": None, "right": None}
        self.tag = tag
        self.pose_st = pose_st

    def __eq__(self, other):
        if not other:
            return False
        if type(other) == int:
            return self.tag == other
        else:
            return self.tag == other.tag

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
        spots = []
        seen.append(self)
        print("get_open_spots for {}: spots are {}".format(self.pips, self.sides))
        if top and top not in seen:
            spots.extend(top.get_open_spots(seen))
        elif top not in seen:
            spots.append((self, "top"))
        if bottom and bottom not in seen:
            spots.extend(bottom.get_open_spots(seen))
        elif bottom not in seen:
            spots.append((self, "bottom"))
        return spots

    def score(self, other):
        """Returns the sum of the pips on a domino.
        We want to get rid of dominoes with higher scores."""
        tocheck = sum(self.pips) + sum(other.pips)
        if tocheck % 5 == 0:
            return tocheck
        else:
            return 0


    def get_domino_direction(self):
        temp = self.pose_st
        quats = temp.pose.orientation
        if quats.z > 0:
            return "L"
        else:
            return "R"

    def get_location_to_move_to(self, side):
        """Returns the offset from this domino's origin to apply to a domino if placing it at side side, specified in this domino's reference frame."""
        temp = PoseStamped()
        temp.pose.position = copy.deepcopy(self.pose_st.pose.position)
        temp.header.frame_id = "base"
        if self.get_domino_direction() == "L":
            if side == "top":
                temp.pose.position.y += VERT_VERT_OFFSET
            if side == "bottom":
                temp.pose.position.y -= VERT_VERT_OFFSET
        if self.get_domino_direction() == "R":
            if side == "top":
                temp.pose.position.y -= VERT_VERT_OFFSET
            if side == "bottom":
                temp.pose.position.y += VERT_VERT_OFFSET
        return temp

Player()
