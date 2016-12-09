#!/usr/bin/env python2

#TODO: lots of imports to call local services, as well as be a node.


"""Currently, this is an OOP skeleton that we should put the ROS in.
CTRL+F your name in all caps for anything I asked you to do.
"""



# Constants to figure out
VERT_VERT_OFFSET = 69694206969
HORIZ_VER_OFFSET = 1337
HAND_SPACE_OFFSET = 0xdaddb0dd

class Player:
    def __init__(self, hand, root):
        self.hand = hand
        self.root = root
        self.turns_taken = 0
        self.seen = []
        self.game_init()

    def game_init(self):
        #Find hand AR tag and populate both seen and hand with the dominoes in our hand.
        pass
        self.hand_base = unmade_get_hand_base

    def pick_from_boneyard(self):
        """Wait for a player to give us a domino"""
        # Make baxter signal that he can't make a move, probably with a nod or a cute audio file.
        domino = self.get_next_domino()
        hand.append(domino)
        hand_coords = self.hand_base
        # This assumes that Baxter-s hand starts in the closest-left corner of the table and it grows to the right.
        hand_coords[1] = hand_coords[1] + len(hand) * HAND_SPACE_OFFSEt
        self.move_domino(domino, self.hand_base - (0, 

    def game_loop(self):
        game_ended = False
        self.game_init()
        while not game_ended:
            while not self.seen % 4 == 0:
                self.get_next_domino()
                ## Make a move:
                self.turns_taken += 1
                if not self.turns_taken % 4:
                    self.make_move()

    def make_move(self):
        # Compute the move to get:
        open_spots = self.root.get_open_spots
        assert(open_spots)
        move = self.best_move_greedy(self.hand, self.open_spots)
        if move[0] == -1:
            # If we can't find a move, we draw.
            self.pick_from_boneyard()
            self.turns_taken += 1
            return
        domino_to_move = self.hand[move[0]]
        domino_to_move_to = open_spots[move[1]]
        if move[2] == "bottom":
            #ROBERT: figure out the logic on these spins, and make an IK service to call them.
            unmade_spin_domino()
        move_to = domino_to_move_to.get_location_from_domino(move[3])
        self.move_domino(domino_to_move, move_to)

    def move_domino(self, domino_to_move, move_to):
        #ROBERT: change the domino coordinates to the world coordinates
        move_to = unmade_translate_coords(move_to)
        pick_and_place(domino_to_move.location, move_to)
        # We don't have to add the domino to seen because we've already seen it in our hand.
        self.turns_taken += 1

    def get_next_domino(self):
        while 1:
            ## Find new dominoes:
            # TODO HENRY:
            dominoes = CV_SENSE
            newdom = None
            for domino in dominoes:
                convert_CV_to_domino
                if not domino.pips in seen:
                    #If we're sensing multiple turns taken, we're doing something wrong.
                    if newdom:
                        raise Exception
                    newdom = domino
                    seen.add(domino.pips)
            return newdom

    def CV_sense(self):
        # TODO: HENRY:
        # make this call the CV service and return a list of dominoes.
        # The dominoes can have any format, but make sure to either specify them here or in the CV engine.
        # We need this information for each domino:
        # num pips, orientation, corresponding AR tag
        # (position on the screen is OK if we can find functionality to get that AR tag, but that seems like kind of a toss-up)
        pass

    def best_move_greedy(hand, open_spots):
        hiscore = 0
        best_play = (-1, -1, "ERR", "ERR") #Position in hand, position in open_spots, side of the moving domino, side of the stationary domino
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
    def __init__(self, pips, location):
        # The syntax for pips is (top pip, bottom pip)
        self.pips = pips
        self.sides = {"top": None, "bottom": None, "left": None, "right": None}

    def place(self, otherdom, pos):
        """ Places otherdom at this domino's pos"""
        assert not self.sides[pos]
        self.sides[pos] = otherdom
        otherdom.sides["bottom"] = self

    def get_open_spots(self):
        """Currently, we're operating on a simplified domino model wherein dominoes can only be in a line.
        This just translates to finding the ends of a linked list."""
        top = self.sides["top"]
        bottom = self.sides["bottom"]
        #TODO: remember how to recursion
        if top:
            spots.append(top.get_open_spots()[0])
        else:
            spots.append(self.pips[0])
        if bottom:
            spots.append(bottom.get_open_spots()[1])
        else:
            spots.append(self.pips[1])

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
        if "side" == "top":
            return (VERT_VERT_OFFSET, 0 0)
        if "side" == "bottom":
            return (-VERT_VERT_OFFSET, 0, 0)
        if "side" == "left":
            return (0, HORIZ_VERT_OFFSET, 0)
        if "side" == "right":
            return (0, -HORIZ_VERT_OFFSET, 0)
