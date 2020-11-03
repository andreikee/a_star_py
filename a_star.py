import numpy as np
from heapq import heappush as push, heappop as pop

def dist_yx(p1_yx, p2_yx):
    p1 = np.array(p1_yx)
    p2 = np.array(p2_yx)
    return np.linalg.norm(p2 - p1, ord=2)

def sum_yx(p1_yx, p2_yx):
    return (p1_yx[0] + p2_yx[0], p1_yx[1] + p2_yx[1])

def validate_point(p_yx, p_0_limit_yx=(0, 0), p_end_limit_yx=None):
    if p_end_limit_yx is None:
        p_end_limit_yx = p_yx
    return p_yx[0] >= p_0_limit_yx[0] and \
           p_yx[1] >= p_0_limit_yx[1] and \
           p_yx[0] <= p_end_limit_yx[0] and \
           p_yx[1] <= p_end_limit_yx[1]

def neighbors(p_yx, p_0_limit_yx=(0, 0), p_end_limit_yx=None):
    delta = [
             (1, 0),
             (0, 1),
             (-1, 0),
             (0, -1),
             (1, 1),
             (1, -1),
             (-1, 1),
             (-1, -1)
             ]

    nbrs = [sum_yx(p_yx, d) for d in delta]
    nbrs = [n for n in nbrs if validate_point(n, p_0_limit_yx, p_end_limit_yx)]
    return nbrs

def path(cameFrom, curr):
    p = []
    c = curr
    while c in cameFrom.keys():
        c = cameFrom[c]
        p.append(c)
    return p

def search(grid, start_yx, goal_yx):
    hp = []
    closed = []
    cameFrom = {}
    ncols = len(grid[0]) - 1
    nrows = len(grid) - 1
    g_inf = 1000


    h = dist_yx(start_yx, goal_yx)
    print(h)
    g = {start_yx: 0}
    f = {start_yx: h}

    push(hp, (f.get(start_yx, g_inf), start_yx))
    while hp:
        print(f'In heap {len(hp)} item(s)')
        curr = pop(hp)[-1]
        
        if curr == goal_yx:
            print('*'*100)
            p = path(cameFrom, curr)[::-1]
            print(p)
            return p
        
        if curr in closed:
            continue
        
        closed.append(curr)
        nbrs = neighbors(curr, (0, 0), (nrows, ncols))

        print()
        print('Neighbors:')
        print('-'*40)
        for nbr in nbrs:
            new_g = g.get(curr, g_inf) + dist_yx(curr, nbr) * grid[nbr[0]][nbr[1]] * 10
            print(f'g.get(curr, g_inf) = {g.get(curr, g_inf)}')
            print(f'dist_yx(curr, nbr) = {dist_yx(curr, nbr) * grid[nbr[0]][nbr[1]]}')
            print(f'New G {curr}-{nbr} = {new_g}')
            if new_g < g.get(nbr, g_inf):
                cameFrom[nbr] = curr
                g[nbr] = new_g
                f[nbr] = g.get(nbr, g_inf) + dist_yx(goal_yx, nbr)
                print(f'==> dist_yx(goal_yx, nbr) = {dist_yx(goal_yx, nbr)}')
                push(hp, (f.get(nbr, g_inf), nbr))


        print(nbrs)

        print()
        print(f'Closed: {closed}')
        print(f'g = {g}')
        print(f'f = {f}')
        print(f'cameFrom = {cameFrom}')

if __name__ == '__main__':

    grid = [
            [0, 1, 0, 0, 0, 0],  # 0
            [0, 1, 0, 0, 0, 0],  # 1
            [0, 1, 0, 0, 0, 0],  # 2
            [0, 1, 0, 0, 1, 0],  # 3
            [0, 0, 0, 0, 1, 0],  # 4
        #    0  1  2  3  4  5
            ]

    goal = (len(grid)-1, len(grid[0])-1)
    search(grid,
           (0, 0),
        #    (goal[0]-2, goal[1]-2),
           (goal[0], goal[1]),
           )
