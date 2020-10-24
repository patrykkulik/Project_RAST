#
# grid_field3.py
#
# Created on 28/05/2020
#
#

import numpy as np

WALL = '#'
PASSABLE = '.'

OPEN = 'o'

def distance3(a, b):
    """ Calculate the heuristic between points a and b in a 3D space"""

    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

class SquareGrid3:
    def __init__(self, length, width, height):
        self.length = length
        self.width = width
        self.height = height

        self.available_nodes = set()

        for y in range(0, self.width):
            for x in range(0, self.length):
                for z in range(0, self.height):
                    self.available_nodes.update([(x, y, z)])

        self.walls = set()
        self.openings = set()

    def in_bounds(self, id):
        (x, y, z) = id
        return 0 <= x <= self.length-1 and 0 <= y <= self.width-1 and 0 <= z <= self.height-1


    def cost(self, from_node, to_node):
        if from_node in self.walls or to_node in self.walls:
            return float('inf')
        else:  # calculates Euclidean distance
            del_x = from_node[0] - to_node[0]
            del_y = from_node[1] - to_node[1]
            del_z = from_node[2] - to_node[2]
            return np.sqrt(del_x**2 + del_y**2 + del_z**2)

    def neighbors(self, id):
        (x, y, z) = id

        nodes = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
        neighbors = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        triples = []

        for delx in [0.125, 0.25, 0.5, 1]:
            for dely in [0.125, 0.25, 0.5, 1]:
                for delz in [0.125, 0.25, 0.5, 1]:
                    triples.append([delx, dely, delz])

        triples.sort(key=lambda triple: triple[0] + triple[1] + triple[2])

        for t in triples:
            (delx, dely, delz) = t

            results = [(x - delx, y, z), (x - delx, y + dely, z), (x, y + dely, z), (x + delx, y + dely, z), (x + delx, y, z),
                       (x + delx, y - dely, z), (x, y - dely, z), (x - delx, y - dely, z),

                       (x - delx, y, z + delz), (x - delx, y + dely, z + delz), (x, y + dely, z + delz), (x + delx, y + dely, z + delz),
                       (x + delx, y, z + delz), (x + delx, y - dely, z + delz), (x, y - dely, z + delz),
                       (x - delx, y - dely, z + delz), (x, y, z + delz),

                       (x - delx, y, z - delz), (x - delx, y + dely, z - delz), (x, y + dely, z - delz),
                       (x + delx, y + dely, z - delz),
                       (x + delx, y, z - delz), (x + delx, y - dely, z - delz), (x, y - dely, z - delz),
                       (x - delx, y - dely, z - delz), (x, y, z - delz)
                       ]

            for node in nodes:
                if results[node] in self.available_nodes:
                    neighbors[node] = results[node]
                    nodes.remove(node)


        neighbors = list(filter(lambda a: type(a) == tuple, neighbors))

        neighbors = filter(self.in_bounds, neighbors)
        return list(neighbors)

    def observe(self, position, obs_range=1.9):
        ret_dict = {}

        for a in self.available_nodes:
            if distance3(a, position) <= obs_range:
                a = tuple(np.round(a, 4))

                ret_dict[a] = PASSABLE

        for w in self.walls:
            # if w[0] >= position[0]-obs_range and w[0] <= position[0]+obs_range and w[1] >= position[1] - obs_range and w[1] <= position[1]+obs_range:
            if distance3(w, position) <= obs_range:
                w = tuple(np.round(w, 4))
                ret_dict[w] = WALL

        return ret_dict

class AgentViewGrid3(SquareGrid3):  # How the robot sees the grid
    def new_walls(self, observation):
        walls_in_obs = {node for node, nodeType in observation.items()
                        if nodeType == WALL}
        return walls_in_obs - self.walls

    def update_walls(self, new_walls):
        self.walls.update(new_walls)