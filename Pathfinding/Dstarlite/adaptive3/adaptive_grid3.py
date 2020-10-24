import numpy as np

WALL = '#'
PASSABLE = '.'
OPEN = 'o'

HUMAN = 'h'
WINDOW = 'w'

def distance3(a, b):
    """ Calculate the heuristic between points a and b in a 3D space"""

    (x1, y1, z1) = a
    (x2, y2, z2) = b

    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def distance2(a, b):
    """ Calculate the heuristic between points a and b in a 3D space"""

    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def distanceZ(a, b):
    """ Calculate the heuristic between points a and b in a 3D space"""

    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return np.sqrt((z1 - z2) ** 2)

def angle(from_node, to_node):
    return np.rad2deg(np.arctan2((to_node[1] - from_node[1]), (to_node[0] - from_node[0])))

class SquareGrid3:
    def __init__(self, length, width, height):
        self.length = length
        self.width = width
        self.height = height
        self.available_nodes = set()

        for z in range(0, self.height):
            for y in range(0, self.width):
                for x in range(0, self.length):
                    self.available_nodes.update([(x, y, z)])

        self.walls = set()
        self.openings = set()

        self.obstacles = set()

        self.triples = []
        for delx in [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]:
            for dely in [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]:
                for delz in [0.5, 1]:
                    self.triples.append([delx, dely, delz])
        self.triples.sort(key=lambda triple: triple[0] + triple[1] + triple[2])

        self.triples_adapt = []
        delz = 0
        for delx in [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]:
            for dely in [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]:
                self.triples_adapt.append([delx, dely, delz])
        self.triples_adapt.sort(key=lambda triple: triple[0] + triple[1] + triple[2])


    def in_bounds(self, id):
        (x, y, z) = id
        return 0 <= x < self.length and 0 <= y < self.width and 0 <= z < self.height

    def cost(self, from_node, to_node):
        if from_node in self.walls or to_node in self.walls:
            return float('inf')
        else:
            return distance3(from_node, to_node)

    def neighbors(self, id):

        if id in self.openings:
            id = tuple((np.round(id[0]*2)/2, np.round(id[1] * 2) / 2, np.round(id[2] * 2) / 2))

        (x, y, z) = id
        nodes = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
        neighbors = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        for t in self.triples:
            to_remove = []

            (delx, dely, delz) = t
            results = [(x - delx, y, z), (x - delx, y + dely, z), (x, y + dely, z), (x + delx, y + dely, z),
                       (x + delx, y, z),
                       (x + delx, y - dely, z), (x, y - dely, z), (x - delx, y - dely, z),
                       (x - delx, y, z + delz), (x - delx, y + dely, z + delz), (x, y + dely, z + delz),
                       (x + delx, y + dely, z + delz),
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
                    to_remove.append(node)

            for i in to_remove:
                nodes.remove(i)



        neighbors = list(filter(lambda a: type(a) == tuple, neighbors))
        neighbors = filter(self.in_bounds, neighbors)

        neighbors = list(neighbors)

        for opening in self.openings:
            if distance3(opening, id) < 0.5*np.sqrt(3):
                neighbors.append(opening)

        return neighbors

    def neighbors_adapt(self, id):
        (x, y, z) = id
        nodes = [0, 1, 2, 3, 4, 5, 6, 7]
        neighbors = [0, 0, 0, 0, 0, 0, 0, 0]

        for t in self.triples_adapt:
            to_remove = []
            (delx, dely, delz) = t
            results = [(x - delx, y, z), (x - delx, y + dely, z), (x, y + dely, z), (x + delx, y + dely, z),
                       (x + delx, y, z),
                       (x + delx, y - dely, z), (x, y - dely, z), (x - delx, y - dely, z)]
            for node in nodes:
                if results[node] in self.available_nodes:
                    neighbors[node] = results[node]
                    to_remove.append(node)

            for i in to_remove:
                nodes.remove(i)

        neighbors = list(filter(lambda a: type(a) == tuple, neighbors))
        neighbors = filter(self.in_bounds, neighbors)

        return list(neighbors)

    def observe(self, position, orientation, obs_range=2):
        (px, py, pz) = position
        ret_dict = {}

        for a in self.available_nodes:
            if distance2(a, position) <= obs_range and distanceZ(a, position) <= 1 and orientation - 45 < angle(position, a) < orientation + 45:
                a = tuple(np.round(a, 4))
                ret_dict[a] = PASSABLE

        for w in self.walls:
            if distance2(w, position) <= obs_range and distanceZ(w, position) <= 1 and orientation - 45 < angle(position, w) < orientation + 45:
                w = tuple(np.round(w, 4))
                ret_dict[w] = WALL

        return ret_dict


    def observe_cv(self, position, orientation, obs_range=2.0):
        (px, py, pz) = position

        ret_dict = {}

        # Get a CV input and return 'w' for window and 'h' for human

        ret_dict[(1.5, 3.55, 1.21, 90)] = WINDOW


        return ret_dict


class AgentViewGrid3(SquareGrid3):  # How the robot sees the grid

    def new_walls(self, observation):
        walls_in_obs = {node for node, nodetype in observation.items()
                        if nodetype == WALL}
        return walls_in_obs - self.walls

    def new_openings(self, observation):
        openings_in_obs = {node for node, nodetype in observation.items()
                           if nodetype == OPEN}

        return openings_in_obs - self.openings

    def update_walls(self, new_walls):
        self.walls.update(new_walls)

