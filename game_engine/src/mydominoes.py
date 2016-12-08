#!/usr/bin/env python2
# Dominoes are represented as tuples of (lower pips, upper pips).
def best_move_greedy(hand, open_spots):
    hiscore = 0
    best_play = (-1, -1) #Position in hand, position in open_spots.
    for spot in range(len(open_spots)):
        for i in range(len(hand)):
            curr = hand[i]
            currscore= score(curr)
            if currscore < hiscore:
                continue
            if curr[0] == open_spots[spot][0]:
                hiscore = currscore
                best_play = (i, spot)
            elif curr[0] == open_spots[spot][1]:
                hiscore = currscore
                best_play = (i, spot)
            elif curr[1] == open_spots[spot][0]:
                hiscore = currscore
                best_play = (i, spot)
            elif curr[1] == open_spots[spot][1]:
                hiscore = currscore
                best_play = (i, spot)
    return best_play

def score(domino):
    """Returns the sum of the pips on a domino.
    We want to get rid of dominoes with higher scores."""
    return domino[0] + domino[1]

# Class for keeping board state:
class Domino:
    def __init__(self, pips):
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
        spots = []
        top = self.sides["top"]
        bottom = self.sides["bottom"]
        #TODO: remember how to recursion
        if not top:
            spots.append(top.get_open_spots()[0])
        if not bottom:
            spots.append(bottom.get_open_spots()[1])
