import mydominoes

def test():
    if mydominoes.best_move_greedy([(3,4), (1,5), (2, 3)], [(2, 3)]) != 0:
        print("ERR at 0")
    if mydominoes.best_move_greedy([(1,5), (2, 3)], [(2, 3)]) != 0:
        print("ERR at 1")

test()
