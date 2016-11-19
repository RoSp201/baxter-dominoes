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
