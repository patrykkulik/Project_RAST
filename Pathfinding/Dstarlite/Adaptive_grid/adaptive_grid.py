import numpy as np
from functools import partial


WALL = '#'
PASSABLE = '.'
OPEN = 'o'
def distance(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.available_nodes = set()

        for y in range(0, self.height):
            for x in range(0, self.width):
                self.available_nodes.update([(x, y)])

        self.walls = set()
        self.openings = set()

        self.obstacles = set()

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x <= self.width-1 and 0 <= y <= self.height-1

    def cost(self, from_node, to_node):
        if from_node in self.walls or to_node in self.walls:
            return float('inf')
        else:
            return distance(from_node, to_node)

    def neighbors(self, id):
        # results = self.observe(id, obs_range=1.5, neighbor=True)

        (x, y) = id

        nodes = [0, 1, 2, 3, 4, 5, 6, 7]
        neighbors = [0, 0, 0, 0, 0, 0, 0, 0]
        pairs = []

        for delx in [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]:
            for dely in [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]:
                pairs.append([delx, dely])

        pairs.sort(key=lambda pair: pair[0] + pair[1])


        for p in pairs:
            to_remove = []
            (delx, dely) = p
            results = [(x - delx, y), (x - delx, y + dely), (x, y + dely), (x + delx, y + dely), (x + delx, y), (x + delx, y - dely), (x, y - dely),
               (x - delx, y - dely)]

            for node in nodes:
                if results[node] in self.available_nodes:
                    neighbors[node] = results[node]
                    to_remove.append(node)

            for i in to_remove:
                nodes.remove(i)


        # results_full = [(x - 1, y), (x - 1, y + 1), (x, y + 1), (x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y - 1),
        #            (x - 1, y - 1)]
        # results_half = [(x - 0.5, y), (x - 0.5, y + 0.5), (x, y + 0.5), (x + 0.5, y + 0.5), (x + 0.5, y),
        #                 (x + 0.5, y - 0.5), (x, y - 0.5),
        #                 (x - 0.5, y - 0.5)]
        # results_quarter = [(x - 0.25, y), (x - 0.25, y + 0.25), (x, y + 0.25), (x + 0.25, y + 0.25), (x + 0.25, y),
        #                 (x + 0.25, y - 0.25), (x, y - 0.25),
        #                 (x - 0.25, y - 0.25)]
        #
        # results_eighth = [(x - 0.125, y), (x - 0.125, y + 0.125), (x, y + 0.125), (x + 0.125, y + 0.125), (x + 0.125, y),
        #                 (x + 0.125, y - 0.125), (x, y - 0.125),
        #                 (x - 0.125, y - 0.125)]
        #
        # results = [0, 0, 0, 0, 0, 0, 0, 0]
        #
        # for i in range(len(results_full)):
        #     if results_eighth[i] in self.available_nodes:
        #         results[i] = results_eighth[i]
        #
        #     elif results_quarter[i] in self.available_nodes and x % 0.25 == 0 and y % 0.25 == 0:
        #         results[i] = results_quarter[i]
        #
        #     elif results_half[i] in self.available_nodes and x % 0.5 == 0 and y % 0.5 == 0:
        #         results[i] = results_half[i]
        #
        #     elif results_full[i] in self.available_nodes and x % 1 == 0 and y % 1 == 0:
        #         results[i] = results_full[i]
        #
        #     elif i == 0:
        #         if y % 0.125 == 0:
        #             dely = 0.25
        #         elif y % 0.25 == 0:
        #             dely = 0.5
        #         elif y % 0.5 == 0:
        #             dely = 0.5
        #
        #         if np.round(y) == np.ceil(y):
        #             py = np.round(y) - dely
        #         elif np.round(y) == np.floor(y):
        #             py = np.round(y) + dely
        #
        #         if (x - 0.25, py + dely) in self.available_nodes and (x - 0.25, py - dely) in self.available_nodes:
        #             results[1] = (x - 0.25, py + dely)
        #             results[7] = (x - 0.25, py - dely)
        #             if (x - 0.25, py) in self.available_nodes:
        #                 results[0] = (x - 0.25, py)
        #         elif (x - 0.5, py + dely) in self.available_nodes and (x - 0.5, py - dely) in self.available_nodes:
        #             results[1] = (x - 0.5, py + dely)
        #             results[7] = (x - 0.5, py - dely)
        #             if (x - 0.5, py) in self.available_nodes:
        #                 results[0] = (x - 0.5, py)
        #         elif (x - 1, py + dely) in self.available_nodes and (x - 1, py - dely) in self.available_nodes:
        #             results[1] = (x - 1, py + dely)
        #             results[7] = (x - 1, py - dely)
        #             if (x - 1, py) in self.available_nodes:
        #                 results[0] = (x - 1, py)
        #
        #     elif i == 2:
        #         # delx = 0.25
        #         if x % 0.125 == 0:
        #             delx = 0.25
        #         elif x % 0.25 == 0:
        #             delx = 0.5
        #         elif x % 0.5 == 0:
        #             delx = 0.5
        #
        #
        #         if np.round(x) == np.ceil(x):
        #             px = np.round(x) - delx
        #         elif np.round(x) == np.floor(x):
        #             px = np.round(x) + delx
        #
        #         if (px + delx, y + 0.25) in self.available_nodes and (px - delx, y + 0.25) in self.available_nodes:
        #             results[1] = (px - delx, y + 0.25)
        #             results[3] = (px + delx, y + 0.25)
        #             if (px, y+0.25) in self.available_nodes:
        #                 results[2] = (px, y+0.25)
        #         elif (px + delx, y + 0.5) in self.available_nodes and (px - delx, y + 0.5) in self.available_nodes:
        #             results[1] = (px - delx, y + 0.5)
        #             results[3] = (px + delx, y + 0.5)
        #             if (px, y+0.5) in self.available_nodes:
        #                 results[2] = (px, y+0.5)
        #         elif (px + delx, y + 1) in self.available_nodes and (px - delx, y + 1) in self.available_nodes:
        #             results[1] = (px - delx, y + 1)
        #             results[3] = (px + delx, y + 1)
        #             if (px, y+1) in self.available_nodes:
        #                 results[2] = (px, y+1)
        #
        #     elif i == 4:
        #         if y % 0.125 == 0:
        #             dely = 0.25
        #         elif y % 0.25 == 0:
        #             dely = 0.5
        #         elif y % 0.5 == 0:
        #             dely = 0.5
        #
        #         if np.round(y) == np.ceil(y):
        #             py = np.round(y) - dely
        #         elif np.round(y) == np.floor(y):
        #             py = np.round(y) + dely
        #
        #         if (x + 0.25, py + dely) in self.available_nodes and (x + 0.25, py - dely) in self.available_nodes:
        #             results[5] = (x + 0.25, py + dely)
        #             results[3] = (x + 0.25, py - dely)
        #             if (x + 0.25, py) in self.available_nodes:
        #                 results[4] = (x + 0.25, py)
        #         elif (x + 0.5, py + dely) in self.available_nodes and (x + 0.5, py - dely) in self.available_nodes:
        #             results[5] = (x + 0.5, py + dely)
        #             results[3] = (x + 0.5, py - dely)
        #             if (x + 0.5, py) in self.available_nodes:
        #                 results[4] = (x + 0.5, py)
        #         elif (x + 1, py + dely) in self.available_nodes and (x + 1, py - dely) in self.available_nodes:
        #             results[5] = (x + 1, py + dely)
        #             results[3] = (x + 1, py - dely)
        #             if (x + 1, py) in self.available_nodes:
        #                 results[4] = (x + 1, py)
        #
        #     elif i == 6:
        #
        #         if x % 0.125 == 0:
        #             delx = 0.25
        #         elif x % 0.25 == 0:
        #             delx = 0.5
        #         elif x % 0.5 == 0:
        #             delx = 0.5
        #
        #         if np.round(x) == np.ceil(x):
        #             px = np.round(x) - delx
        #         elif np.round(x) == np.floor(x):
        #             px = np.round(x) + delx
        #
        #         if (px + delx, y - 0.25) in self.available_nodes and (px - delx, y - 0.25) in self.available_nodes:
        #             results[1] = (px - delx, y - 0.25)
        #             results[3] = (px + delx, y - 0.25)
        #             if (px, y + 0.25) in self.available_nodes:
        #                 results[6] = (px, y - 0.25)
        #         elif (px + delx, y - 0.5) in self.available_nodes and (px - delx, y - 0.5) in self.available_nodes:
        #             results[1] = (px - delx, y - 0.5)
        #             results[3] = (px + delx, y - 0.5)
        #             if (px, y - 0.5) in self.available_nodes:
        #                 results[6] = (px, y - 0.5)
        #         elif (px + delx, y - 1) in self.available_nodes and (px - delx, y - 1) in self.available_nodes:
        #             results[1] = (px - delx, y - 1)
        #             results[3] = (px + delx, y - 1)
        #             if (px, y - 1) in self.available_nodes:
        #                 results[6] = (px, y - 1)
        #
        # for i in range(len(results_full)):
        #     if results_eighth[i] in self.available_nodes:
        #         results[i] = results_eighth[i]
        #
        #     elif results_quarter[i] in self.available_nodes and x % 0.25 == 0 and y % 0.25 == 0:
        #         results[i] = results_quarter[i]
        #
        #     elif results_half[i] in self.available_nodes and x % 0.5 == 0 and y % 0.5 == 0:
        #         results[i] = results_half[i]
        #
        #     elif results_full[i] in self.available_nodes and x % 1 == 0 and y % 1 == 0:
        #         results[i] = results_full[i]

        neighbors = list(filter(lambda a: type(a) == tuple, neighbors))

        neighbors = filter(self.in_bounds, neighbors)
        return list(neighbors)
        # return results

    def observe(self, position, obs_range=2.0, neighbor = False):
        (px, py) = position
        nodes = []
        ret_dict = {}
        lim_pos_x = {}
        lim_pos_y = {}
        lim_neg_x = {}
        lim_neg_y = {}
        neighbors = []

        for obstacle in self.obstacles:
            if obstacle[0][0] == obstacle[0][1]:
                for y in np.arange(obstacle[1][0], obstacle[1][1] + 0.125, 0.125):
                    y = np.round(y, 4)
                    if obstacle[0][0] > px:
                        lim_pos_x[y] = min(lim_pos_x.get(y, float('inf')), obstacle[0][0])
                    elif obstacle[0][0] < px:
                        lim_neg_x[y] = max(lim_neg_x.get(y, 0), obstacle[0][0])


            elif obstacle[1][0] == obstacle[1][1]:
                for x in np.arange(obstacle[0][0], obstacle[0][1] + 0.125, 0.125):
                    x = np.round(x, 4)
                    if obstacle[1][0] > py:
                        lim_pos_y[x] = min(lim_pos_y.get(x, float('inf')), obstacle[1][1])
                    elif obstacle[1][0] < px:
                        lim_neg_y[x] = max(lim_neg_y.get(x, 0), obstacle[1][1])


        for a in self.available_nodes:
            if distance(a, position) <= obs_range:
                a = tuple(np.round(a, 4))
                # if a[0] > px:
                #     if a[0] > lim_pos_x.get(a[1], float('inf')):
                #         continue
                # elif a[0] < px:
                #     if a[0] < lim_neg_x.get(a[1], 0):
                #         continue
                # if a[1] > py:
                #     if a[1] > lim_pos_y.get(a[0], float('inf')):
                #         continue
                # elif a[1] < py:
                #     if a[1] < lim_neg_y.get(a[0], 0):
                #         continue
                ret_dict[a] = PASSABLE
                neighbors.append(a)

        for w in self.walls:
            # if w[0] >= position[0]-obs_range and w[0] <= position[0]+obs_range and w[1] >= position[1] - obs_range and w[1] <= position[1]+obs_range:
            if distance(w, position) <= obs_range:
                w = tuple(np.round(w, 4))
                # if w[0] > px:
                #     if w[0] > lim_pos_x.get(w[1], float('inf')):
                #         continue
                # elif w[0] < px:
                #     if w[0] < lim_neg_x.get(w[1], 0):
                #         continue
                # if w[1] > py:
                #     if w[1] > lim_pos_y.get(w[0], float('inf')):
                #         continue
                # elif w[1] < py:
                #     if w[1] < lim_neg_y.get(w[0], 0):
                #         continue
                ret_dict[w] = WALL

        for o in self.openings:
            if distance(o, position) <= obs_range:
                ret_dict[o] = OPEN

        if neighbor:
            return neighbors
        else:
            return ret_dict


class AgentViewGrid(SquareGrid):  # How the robot sees the grid

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
