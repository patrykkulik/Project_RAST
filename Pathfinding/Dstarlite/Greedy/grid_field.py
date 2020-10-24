import numpy as np

WALL = '#'
PASSABLE = '.'
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = set()

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def cost(self, from_node, to_node):
        if from_node in self.walls or to_node in self.walls:
            return float('inf')
        elif from_node[0] == to_node[0]:
            return abs(from_node[1] - to_node[1])
        elif from_node[1] == to_node[1]:
            return abs(from_node[0] - to_node[0])
        else:  # if the node is somewhere in between the previous options
            del_x = from_node[0] - to_node[0]
            del_y = from_node[1] - to_node[1]
            return np.sqrt(del_x**2 + del_y**2)

    def neighbors(self, id):
        (x, y) = id
        x = np.round(x)
        y = np.round(y)
        results = [(x - 1, y), (x-1, y+1), (x, y + 1), (x+1,y+1), (x + 1, y), (x+1, y-1), (x, y - 1), (x-1, y-1)]
        results.reverse()
        # if (x + y) % 2 == 0: results.reverse()  # aesthetics
        results = filter(self.in_bounds, results)
        return list(results)

        # (x, y) = id
        # # results = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        # results = []
        # for i in range(-10,11):
        #     results.append((x+0.1*i, y+1))
        #     results.append((x + 0.1 * i, y - 1))
        # for i in range(-9,10):
        #     results.append((x+1, y+0.1*i))
        #     results.append((x-1, y + 0.1 * i))
        #
        # if (x + y) % 2 == 0: results.reverse()  # aesthetics
        # results = filter(self.in_bounds, results)
        # return list(results)

    def observe(self, position, obs_range=2):
        (px, py) = position
        nodes = []
        for w in self.walls:
            # if w[0] >= position[0]-obs_range and w[0] <= position[0]+obs_range and w[1] >= position[1] - obs_range and w[1] <= position[1]+obs_range:
            if heuristic(w, position) <= obs_range:
                nodes.append(w)
        return {node: WALL for node in nodes}


class AgentViewGrid(SquareGrid):  # How the robot sees the grid

    def new_walls(self, observation):
        walls_in_obs = {node for node, nodetype in observation.items()
                        if nodetype == WALL}
        return walls_in_obs - self.walls

    def update_walls(self, new_walls):
        self.walls.update(new_walls)