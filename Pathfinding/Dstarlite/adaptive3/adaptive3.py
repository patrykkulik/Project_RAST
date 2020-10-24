from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from functools import partial
from mpl_toolkits.mplot3d import Axes3D
import time

# Import needed functions from other scripts:
from utility_test3 import draw_grid3, label_grid3
from utility_test3 import grid_from_string3
from priority_queue import PriorityQueue
from adaptive_grid3 import AgentViewGrid3, SquareGrid3


def distance3(a, b):
    """ Calculate a straight line distance between points a and b on a 2D plane """

    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def distance2(a, b):
    """ Calculate the heuristic between points a and b in a 3D space"""

    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def heuristic3(a, b):
    """ Calculate the heuristic between points a and b on a 3D plane """
    # Returns a min distance between two nodes assuming motion is only possible at 0, 45 and 90 degree angles
    (x1, y1, z1) = a
    (x2, y2, z2) = b

    distances = [abs(x1 - x2), abs(y1 - y2), abs(z1 - z2)]
    distances.sort()
    sqrt3dist = distances[0]
    sqrt2dist = distances[1] - sqrt3dist
    sqrt1dist = distances[2] - sqrt2dist - sqrt3dist
    return sqrt3dist * np.sqrt(3) + sqrt2dist * np.sqrt(2) + sqrt1dist


# MARK: algorithm start

class adaptive3(object):
    def __init__(self, graph, start, goal, view_range=1.75):
        # Initialise the Field D* algorithm:

        self.graph = AgentViewGrid3(graph.length, graph.width,
                                    graph.height)  # Define the graph method as an instance of AgentView
        # (the map from the perspective of the robot)
        self.real_graph = graph  # Make the real_graph method equal to the real version of the grid
        self.view_range = view_range  # Define the range of view of the robot

        self.seen = set()  # Create a set of all the seen nodes
        self.orientation = 0

        self.openings = set()
        self.openings_neighbors = {}
        self.human_nodes = set()

        self.Km = 0  # Key modifier. When the robot makes a move, this is incremented so that heuristic? can stay same
        self.real_world_multiplier = 2  # how many meters is one step equivalent to in the real world
        self.real_world_multiplierZ = 1
        self.position = start  # Starting position of the robot
        self.start = start
        self.goal = (500, 500, 500)  # Goal position (fake)
        # Create dictionaries:
        self.G_VALS = {start: 1000,
                       self.goal: float('inf')}
        self.RHS_VALS = {start: 1000, self.goal: 0}  # Dictionary of all rhs values for all of the expanded cells

        self.frontier = PriorityQueue()  # Define the frontier method as an instance of PriorityQueue
        self.frontier.put(self.goal, self.calculate_key(self.goal))  # Append the goal and its key to the frontier

    def initialise_plot(self):
        fig = plt.figure()
        # plt.grid()
        # ax = fig.add_subplot(1, 1, 1, projection='3d')
        # ax_label = fig.add_subplot(1, 2, 2, projection='3d')
        ax_label = 1
        ax = 1

        # ax.set_xlim(0, self.real_world_multiplier * self.graph.length)
        # ax.set_ylim(0, self.real_world_multiplier * self.graph.width)
        # ax.set_zlim(0, self.real_world_multiplierZ * self.graph.height)
        # ax.xaxis.set_major_locator(plt.MultipleLocator(self.real_world_multiplier))
        # ax.yaxis.set_major_locator(plt.MultipleLocator(self.real_world_multiplier))
        # ax.zaxis.set_major_locator(plt.MultipleLocator(self.real_world_multiplierZ))
        # plt.pause(1)
        #
        # for a in self.graph.available_nodes:
        #     ax.plot([self.real_world_multiplier * a[0]], [self.real_world_multiplier * a[1]],
        #             [self.real_world_multiplierZ * a[2]], 'o', color='gray', alpha=0.1)
        #
        # ax.view_init(elev=40, azim=-60)

        return ax, ax_label

    def rhs(self, node):
        """
        Return node's rhs value. If node not in the RHS_VALS dict, return inf. If node is the goal node, return 0
        """
        # rhs value is what the g value of the node should be
        if node == self.goal:
            return 0  # The goal node should always be zero

        elif node in self.human_nodes:
            return 0.5

        elif node == self.start:
            return 1000  # To ensure that the drone comes back to start at the end, the start node should have a
            # constant, high rhs

        elif node == self.position:
            return 1.2 * self.RHS_VALS.get(node, 1)

        else:
            return self.RHS_VALS.get(node, 1)

    def g(self, node):
        """ Return the node's g value. If node not present in the G_VALS dict, or node is an obstacle, return inf """
        # g value is essentially the optimal distance from the current node to the goal node (given the current
        # knowledge of the map)
        if self.obstacle_check3(node):
            return_val = float('inf')
        else:
            return_val = self.G_VALS.get(node, float('inf'))
        return return_val

    def calculate_key(self, node):
        """ Return the key pair for a given node """
        # Key determines the priority of a node on the expansion queue. The lower the key, the closer the node is to the
        # goal. On the priority queue, nodes are expanded and updated according to the priority key
        g_rhs = min([self.g(node), self.rhs(node)])  # Since g and rhs should be equal for an expanded node, a min value
        # is taken to ensure that node is not expanded too late

        return (
            np.round(g_rhs + heuristic3(node, self.position) + self.Km, 4),
            np.round(g_rhs, 4)
        )
        # The key is a combination of the total function and the start distance. A rounding is applied to prevent
        # machine precision from introducing problems

    def obstacle_check3(self, node):
        """ Check if the node is inside an obstacle """
        if node in self.graph.openings:
            # did you mean this
            return False
        # If coordinate in available coordinates, check the vertex
        elif node[0] % 0.125 == 0 and node[1] % 0.125 == 0 and node[2] % 0.125 == 0 \
                and node in self.graph.available_nodes:
            if node in self.graph.walls:
                return True
            else:
                return False
        else:
            return False

    def feasible_motion3(self, from_node, to_node):
        """ Function to check if motion between two nodes is feasible (no obstacle present in between) """
        # Could be improved to make it less convoluted
        return not self.obstacle_check3(to_node)

    def lookahead_cost(self, node, neighbour):
        if node[2] != neighbour[2]:
            c = 4
        else:
            c = 2
        return self.g(neighbour) + c * distance3(neighbour, node)

    def compute_cost(self, node, neighbors):
        """Compute cost function developed by Dave Ferguson and Anthony Stentz
        (https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_4/ferguson_david_2005_4.pdf)
        :return vs: rhs of node s from edge sa <-> sb
         destination: the destination to obtain optimal cost
        """

        cost = partial(self.lookahead_cost, node)
        destination = min(neighbors, key=cost)  # Find the point closest to id in the available nodes set

        while not self.feasible_motion3(node, destination) and len(neighbors) > 0:
            destination = min(neighbors, key=cost)
            neighbors.remove(destination)

        if len(neighbors) != 0:
            vs = self.lookahead_cost(node, destination)
        else:
            vs = float('inf')

        return vs, destination

    def calculate_rhs3(self, node):
        """ Calculate the rhs value of node """
        if self.obstacle_check3(node):  # If node is in an obstacle, rhs is equal to inf
            return float('inf')
        elif node not in self.graph.available_nodes:
            return float('inf')
        else:
            # Compute all local neighbors of the node
            local_neighbors = self.graph.neighbors(node)
            if node in self.openings:
                local_neighbors.append(self.openings_neighbors[node])
            vs, destination = self.compute_cost(node, local_neighbors)  # compute the cost

            return vs

    def update_node(self, node):
        """ Update the rhs value of a node and put in on the frontier if it is inconsistent """

        if node != self.goal and node in self.seen and node != self.start and node not in self.human_nodes:
            self.RHS_VALS[node] = self.calculate_rhs3(node)
        else:
            self.RHS_VALS[node] = self.rhs(node)

        self.frontier.delete(node)  # remove the node from the frontier

        if self.g(node) != self.rhs(node):  # if the node is not consistent, place it back in the frontier
            self.frontier.put(node, self.calculate_key(node))

    def update_nodes(self, nodes):
        """ Update all nodes in the nodes list """
        [self.update_node(n) for n in nodes]

    def compute_shortest_path(self):
        """ Compute shortest path according to the D* lite algorithm """
        # Take the node with smallest priority key on the queue and update its g value and update its neighbors

        last_nodes = deque(maxlen=10)  # List like object that can be appended and popped from both sides.

        # As long as the key in the frontier are lower than at my current position or I'm in the position where g!=rhs
        while self.frontier.first_key() < self.calculate_key(self.position) or self.rhs(self.position) != self.g(
                self.position):

            k_old = self.frontier.first_key()  # The old key is the smallest key from the frontier
            node = self.frontier.pop()  # The node is equal to the node in frontier with smallest key
            last_nodes.append(node)  # Append this node to last_nodes

            if len(last_nodes) == 10 and len(set(last_nodes)) < 3:  # if the last_nodes list is full but contains less
                # than 3 unique nodes
                raise Exception("Fail! Stuck in a loop")

            k_new = self.calculate_key(node)  # Calculate the new key function for the current node

            if k_old < k_new:  # If the old key was better than the new key
                self.frontier.put(node, k_new)  # Append the node with the new key to the frontier

            # If the g(node) > rhs(node) we have a simple situation where we can just set g(node) = rhs(node) and update
            # neighbors
            elif self.g(node) > self.rhs(node):
                self.G_VALS[node] = self.rhs(node)
                if node != self.goal:
                    self.update_nodes(self.graph.neighbors(node))
                    if node == self.position:
                        self.update_node(node)
                else:
                    self.update_nodes(self.graph.neighbors((2, 2, 0)))

                if node in self.openings:
                    self.update_node(self.openings_neighbors[node])
            # If the g(node) < rhs(node) we have a difficult situation where we first have to set the g(node) = inf,
            # after which we move the node and its neighbors back to the frontier. This ensures that the node is not
            # updated more than necessary. After the required nodes move through the queue and this node is being
            # expanded, the situation becomes a simple case where the g(node) is relaxed to its rhs value
            else:
                self.G_VALS[node] = float('inf')
                if node != self.goal:
                    for n in self.graph.neighbors(node):
                        self.update_node(n)

                    # for opening in self.graph.openings:
                    #     if node[0] - 1.5 <= opening[0] <= node[0] + 1.5 and node[1] - 1.5 <= opening[1] <= \
                    #                     node[1] + 1.5:
                    #         self.update_node(opening)

                    self.update_node(node)
                else:
                    self.update_node(node)
                    self.update_nodes(self.graph.neighbors((2, 2, 0)))

                if node in self.openings:
                    self.update_node(self.openings_neighbors[node])

            if self.frontier.empty():
                break

        return self.G_VALS.copy()


    def adapt_grid(self, new_walls, ax, robot_plot=True):
        new_nodes = set()

        for wall in new_walls:
            wall = tuple(np.round(wall, 4))
            # if wall[2] == 2:
            #     ax.plot([self.real_world_multiplier * wall[0]], [self.real_world_multiplier * wall[1]],
            #             [self.real_world_multiplierZ * wall[2]], 'ks', alpha = 0.1)
            # elif 0 < wall[1] < 3.5 and 0 < wall[0] < 1.8 and wall[2]<4:
            #     ax.plot([self.real_world_multiplier * wall[0]], [self.real_world_multiplier * wall[1]],
            #             [self.real_world_multiplierZ * wall[2]], 's', color = 'blue', alpha = 0.05)
            # else:
            #     ax.plot([self.real_world_multiplier * wall[0]], [self.real_world_multiplier * wall[1]],
            #             [self.real_world_multiplierZ * wall[2]], 's', color = 'gray', alpha = 0.05)

            if wall not in self.graph.available_nodes:
                pos = (np.floor(wall[0] * 8) / 8 + 1 / 16,
                       np.floor(wall[1] * 8) / 8 + 1 / 16,
                       np.round(wall[2] * 2) / 2)

                for delx in [-1 / 16, 1 / 16]:
                    for dely in [-1 / 16, 1 / 16]:
                        new_node = (pos[0] + delx, pos[1] + dely, pos[2])
                        if self.graph.in_bounds(new_node):
                            self.graph.available_nodes.update([new_node])
                            self.graph.walls.update([new_node])
                            self.update_node(new_node)
                            if robot_plot:
                                ax.plot([self.real_world_multiplier * new_node[0]],
                                        [self.real_world_multiplier * new_node[1]],
                                        [self.real_world_multiplierZ * new_node[2]],
                                        'o', color='gray')
                            self.seen.update([new_node])
                            new_nodes.update([new_node])


                if np.round(wall[2] * 2) / 2 == np.ceil(wall[2] * 2) / 2:
                    pos_z = np.round(wall[2] * 2) / 2 - 0.5
                elif np.round(wall[2] * 2) / 2 == np.floor(wall[2] * 2) / 2:
                    pos_z = np.round(wall[2] * 2) / 2 + 0.5

                pos = (np.floor(wall[0] * 2) / 2 + 1 / 4,
                       np.floor(wall[1] * 2) / 2 + 1 / 4,
                       pos_z)

                for delx in [-1 / 4, 1 / 4]:
                    for dely in [-1 / 4, 1 / 4]:
                        new_node = (pos[0] + delx, pos[1] + dely, pos[2])
                        if self.graph.in_bounds(new_node):
                            self.graph.available_nodes.update([new_node])
                            self.graph.walls.update([new_node])
                            self.update_node(new_node)
                            if robot_plot:
                                ax.plot([self.real_world_multiplier * new_node[0]],
                                        [self.real_world_multiplier * new_node[1]],
                                        [self.real_world_multiplierZ * new_node[2]],
                                        'o', color='gray')
                            self.seen.update([new_node])
                            new_nodes.update([new_node])
                self.graph.walls.remove(wall)


        for new_node in new_nodes:
            for neighbor in self.graph.neighbors_adapt(new_node):
                if distance2(new_node, neighbor) >= 0.25 and neighbor[2] == new_node[2]:
                    if neighbor in self.graph.walls:
                        px = (neighbor[0] + new_node[0]) / 2
                        py = (neighbor[1] + new_node[1]) / 2
                        pz = (neighbor[2] + new_node[2]) / 2
                        pz = np.round(pz * 2) / 2
                        con_x = False
                        con_y = False

                        if px % 0.125 == 0 and py % 0.125 == 0:
                            open_node = (px, py, pz)
                            self.adapt_grid_helper(open_node, ax)

                        elif px % 0.125 == 0:
                            for dy in [-1 / 16, 1 / 16]:
                                open_node = (px, py + dy, pz)
                                if py + dy == new_node[1] or py + dy == neighbor[1]:
                                    con_y = True
                                self.adapt_grid_helper(open_node, ax)
                        elif py % 0.125 == 0:
                            for dx in [-1 / 16, 1 / 16]:
                                open_node = (px + dx, py, pz)
                                if px + dx == new_node[0] or px + dx == neighbor[0]:
                                    con_x = True

                                self.adapt_grid_helper(open_node, ax)
                        else:
                            for dx in [-1 / 16, 1 / 16]:
                                for dy in [-1 / 16, 1 / 16]:
                                    open_node = (px + dx, py + dy, pz)
                                    if py + dy == new_node[1] or py + dy == neighbor[1]:
                                        con_y = True
                                    if px + dx == new_node[0] or px + dx == neighbor[0]:
                                        con_x = True

                                    self.adapt_grid_helper(open_node, ax)
                        if con_x:
                            open_node_1 = (np.ceil(px), np.floor(py * 8) / 8, pz)
                            open_node_2 = (np.floor(px), np.floor(py * 8) / 8, pz)

                            self.adapt_grid_helper(open_node_1, ax)

                            self.adapt_grid_helper(open_node_2, ax)

                            open_node_3 = (np.ceil(px) + 1, np.floor(py * 8) / 8, pz)
                            open_node_4 = (np.floor(px) - 1, np.floor(py * 8) / 8, pz)

                            self.adapt_grid_helper(open_node_3, ax)

                            self.adapt_grid_helper(open_node_4, ax)

                        # elif neighbor[1] == new_node[1]:
                        elif con_y:
                            open_node_1 = (np.floor(px * 8) / 8, np.ceil(py), pz)
                            open_node_2 = (np.floor(px * 8) / 8, np.floor(py), pz)

                            self.adapt_grid_helper(open_node_1, ax)

                            self.adapt_grid_helper(open_node_2, ax)

                            open_node_3 = (np.floor(px * 8) / 8, np.ceil(py) + 1, pz)
                            open_node_4 = (np.floor(px * 8) / 8, np.floor(py) - 1, pz)

                            self.adapt_grid_helper(open_node_3, ax)

                            self.adapt_grid_helper(open_node_4, ax)



    def adapt_grid_helper(self, open_node, ax):
        if self.graph.in_bounds(open_node):
            self.graph.available_nodes.update([open_node])
            self.update_node(open_node)
            # ax.plot([self.real_world_multiplier * open_node[0]], [self.real_world_multiplier * open_node[1]],
            #         [self.real_world_multiplierZ * open_node[2]], 'o', color='green', alpha=0.1)
            self.seen.update([open_node])


    def create_offset(self, observation_cv, ax):
        for object in list(observation_cv.keys()):
            if observation_cv[object] == 'w':
                # ax.plot([self.real_world_multiplier * object[0]], [self.real_world_multiplier * object[1]],
                #         [self.real_world_multiplierZ * object[2]], 'o', color='orange')

                angle = np.deg2rad(object[3])
                px = object[0]
                py = object[1]
                pz = object[2]

                offset_node = (px + 0.5*np.cos(angle), py + 0.5*np.sin(angle), pz)
                angle_2 = np.deg2rad(object[3] + 180)
                offset_node_2 = (px + 0.5 * np.cos(angle_2), py + 0.5 * np.sin(angle_2), pz)

                if self.graph.in_bounds(offset_node) and self.graph.in_bounds(offset_node_2):
                    self.graph.available_nodes.update([offset_node])
                    self.update_node(offset_node)
                    # ax.plot([self.real_world_multiplier * offset_node[0]], [self.real_world_multiplier * offset_node[1]],
                    #         [self.real_world_multiplierZ * offset_node[2]], 'o', color='purple')
                    self.openings.update([offset_node])
                    self.graph.openings.update([offset_node])



                    self.graph.available_nodes.update([offset_node_2])
                    self.update_node(offset_node_2)
                    # ax.plot([self.real_world_multiplier * offset_node_2[0]], [self.real_world_multiplier * offset_node_2[1]],
                    #         [self.real_world_multiplierZ * offset_node_2[2]], 'o', color='purple')
                    self.openings.update([offset_node_2])
                    self.graph.openings.update([offset_node_2])

                    self.openings_neighbors[offset_node] = offset_node_2
                    self.openings_neighbors[offset_node_2] = offset_node


            if observation_cv[object] == 'h':
                # ax.plot([self.real_world_multiplier * object[0]], [self.real_world_multiplier * object[1]],
                #         [self.real_world_multiplierZ * object[2]], 'o', color='yellow')

                for neighbor in self.graph.neighbors(object):
                    if neighbor not in self.graph.walls:
                        self.human_nodes.update(neighbor)
                        self.update_node(neighbor)

    def initialise(self):
        ############################################################################################
        #ALL OF THIS PART IS DONE BY SLAM AND CV


        # Initialise the plot axes and draw the grid
        ax, ax_label = self.initialise_plot()

        observation = self.real_graph.observe(self.position, self.orientation, self.view_range)  # Observe the range

        walls = self.graph.new_walls(
            observation)  # Define the position of newly observed walls to the graph seen by the robot

        self.graph.update_walls(walls)  # Update the walls on the robot's map

        self.adapt_grid(walls, ax, robot_plot=False)

        for n in list(observation.keys()):
            if n in self.graph.available_nodes:
                self.seen.update([n])

        observation_cv = self.real_graph.observe_cv(self.position, self.orientation, self.view_range)

        # END OF THE PART DONE BY SLAM and CV
        ############################################################################################

        if len(observation_cv) != 0:  # THIS IS BASICALLY SAYING THAT IF CV GAVE AN INPUT RUN THE CREATE_OFFSET
            self.create_offset(observation_cv, ax)

        # Compute the shortest path to the goal based on the current information
        self.compute_shortest_path()

        # label_grid3(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal, self.real_world_multiplier, self.real_world_multiplierZ)
        # draw_grid3(self.graph, ax, self.view_range, self.real_world_multiplier, self.real_world_multiplierZ,
        #            self.orientation,
        #            path=self.position, goal=self.goal)

        return ax, ax_label  # WON'T NEED TO HAVE THIS IN THE REAL CODE

    def move_to_goal(self, ax, ax_label):

        condition = True

        # If the robot becomes stuck in a position with inf g, there is no path that can be taken
        if self.rhs(self.position) == float('inf'):
            raise Exception("No path")

        ############################################################################################
        # ALL OF THIS PART IS DONE BY SLAM AND CV

        observation = self.real_graph.observe(self.position, self.orientation, self.view_range)  # make a new observation
        new_walls = self.graph.new_walls(observation)  # obtain the new walls from the observation

        observation_cv = self.real_graph.observe_cv(self.position, self.orientation, self.view_range)

        # END OF THE PART DONE BY SLAM and CV
        ############################################################################################

        newly_seen = []
        for n in list(observation.keys()):
            if n in self.graph.available_nodes:
                if n not in self.seen:
                    newly_seen.append(n)
                self.seen.update([n])

        if len(observation_cv) != 0:
            self.create_offset(observation_cv, ax)

        # If new walls have been observed, update walls, the key modifier and nodes
        if new_walls:
            self.graph.update_walls(new_walls)

            self.adapt_grid(new_walls, ax, robot_plot=False)

            self.update_node(self.position)
            self.update_node(self.start)
            if len(self.frontier.elements) != 0:
                self.compute_shortest_path()  # Recompute the shortest path

        elif len(self.seen) == len(self.graph.available_nodes):  # If all points have been seen
            if self.position == self.start:
                print("available nodes {}".format(len(self.graph.available_nodes)))
                print("height {}".format(self.graph.height))
                print("width {}".format(self.graph.width))
                pass
            else:
                self.G_VALS = {self.start: float('inf'),
                               self.goal: float('inf')}
                self.RHS_VALS = {self.start: 1000, self.goal: 0}

                self.update_node(self.start)

                if len(self.frontier.elements) != 0:
                    self.compute_shortest_path()  # Recompute the shortest path

        else:
            self.update_node(self.position)
            self.G_VALS[self.position] = self.rhs(self.position)
            for n in self.graph.neighbors(self.position):
                self.update_node(n)
                self.G_VALS[n] = self.rhs(n)

            self.update_node(self.position)
            self.G_VALS[self.position] = self.rhs(self.position)

            # if len(newly_seen) == 0:
            #     keys = list(self.RHS_VALS.keys())
            #     for key in keys:
            #         if \
            #                 (self.position[0] - (self.view_range + 1) <= key[0] <= self.position[
            #                     0] + self.view_range + 1) and (
            #                         self.position[1] - (self.view_range + 1) <= key[1] <= self.position[
            #                     1] + self.view_range + 1) and (
            #                         self.position[2] - (self.view_range + 1) <= key[2] <= self.position[
            #                     2] + self.view_range + 1):
            #             if self.RHS_VALS[key] == 1:
            #                 condition = False
            #                 break

            # Condition only True if the robot does not have unvisited nodes just outside its field of vision
            if condition:
                self.update_node(self.start)
                if len(self.frontier.elements) != 0:
                    self.compute_shortest_path()  # Recompute the shortest path

        # yield self.position, observation, self.graph.walls
        # label_grid3(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal, self.real_world_multiplier, self.real_world_multiplierZ)
        # draw_grid3(self.graph, ax, self.view_range, self.real_world_multiplier, self.real_world_multiplierZ, self.orientation,
        #            path=self.position, goal=self.goal)

        best_step = self.position

        neighbors = self.graph.neighbors(self.position)

        for neighbor in neighbors:
            if self.g(neighbor) + distance3(neighbor, self.position) * 2 < self.g(best_step) + distance3(best_step,
                                                                                                          self.position) * 2:
                best_step = neighbor

        if self.position in self.openings:
            neighbor = self.openings_neighbors[self.position]
            if best_step not in self.openings:
                if self.g(neighbor) + distance3(neighbor, self.position)*2 + 1< self.g(best_step) + distance3(best_step, self.position)*2:
                    best_step = neighbor
            else:
                if self.g(neighbor) + distance3(neighbor, self.position)*2 < self.g(best_step) + distance3(best_step, self.position)*2:
                    best_step = neighbor
                ### AT THIS POINT YOU WOULD CALL A MORPHING SCRIPT

        # is the best step within the visible range?
        best_orientation = np.rad2deg(np.arctan2((best_step[1] - self.position[1]),(best_step[0] - self.position[0])))
        best_orientation = np.round(best_orientation/45)*45

        if best_orientation == self.orientation:
            self.position = best_step

        self.orientation = best_orientation

        if self.position in self.openings:
            self.seen.update([self.position])
        elif self.position in self.human_nodes:
            self.human_nodes = set()
            # AT THIS POINT YOU WOULD DROP PAYLOAD

        return (self.position[0], self.position[1], self.position[2], self.orientation)

if __name__ == "__main__":
    # GRAPH, START, END = grid_from_string("""
    #     o.....
    #     ......
    #     .....X
    #     """)
    #
    # y1 = np.arange(1, 2.2, 0.1)
    #
    # for i in range(len(y1)):
    #     GRAPH.walls.add((3.1, np.round(y1[i], 4)))

    # GRAPH, START, END = grid_from_string("""
    #            .############
    #            .#....#.....#
    #            .##.###.#...#
    #            .#......#...#
    #            .#......#...#
    #            .#......#.###
    #            .#......#...#
    #            .#......#...#
    #            .#......#...#
    #            ..o.....#...#
    #            .#...########
    #            .#......#...#
    #            .#...#..#.X.#
    #            .#...#..#...#
    #            .#########.##
    #            .............
    #             """)

    # GRAPH, START, END = grid_from_string3("""
    #            ########
    #            #......#
    #            #......#
    #            #......#
    #            #......#
    #            #......#
    #            #......#
    #            #o.....#
    #            #......#
    #            #......#
    #            #.....X#
    #            ########
    #             """, 8, 12, 2)
    #
    # y1 = np.arange(1, 7.6, 0.1)
    #
    # for i in range(len(y1)):
    #     GRAPH.walls.add((4.4, y1[i], 0))
    #
    # GRAPH.obstacles.add(((4.4, 4.4, 0), (1, 7.5, 0)))
    #
    # y2 = np.arange(7.5, 11.1, 0.1)
    #
    # for i in range(len(y2)):
    #     GRAPH.walls.add((3.2, y2[i], 0))
    #
    # GRAPH.obstacles.add(((3.2, 3.2, 0), (7.5, 11, 0)))
    #
    # y3 = np.arange(9, 11.1, 0.1)
    #
    # for i in range(len(y3)):
    #     GRAPH.walls.add((1.7, y3[i], 0))
    #
    # GRAPH.obstacles.add(((1.7, 1.7), (9, 11)))
    #
    # y4 = np.arange(0, 11.1, 0.1)
    #
    # for i in range(len(y4)):
    #     GRAPH.walls.add((0, y4[i], 0))
    #     GRAPH.walls.add((7, y4[i], 0))
    #
    # GRAPH.obstacles.add(((0, 0, 0), (0, 11, 0)))
    # GRAPH.obstacles.add(((7, 7, 0), (0, 11, 0)))
    #
    # x1 = np.arange(3, 5.3, 0.1)
    #
    # for i in range(len(x1)):
    #     GRAPH.walls.add((x1[i], 7.5, 0))
    #
    # GRAPH.obstacles.add(((3, 5.2, 0), (7.5, 7.5, 0)))
    #
    # x2 = np.arange(6, 7.1, 0.1)
    #
    # for i in range(len(x2)):
    #     GRAPH.walls.add((x2[i], 7.5, 0))
    #
    # GRAPH.obstacles.add(((6, 7.1, 0), (7.5, 7.5, 0)))
    #
    # x3 = np.arange(0, 7.1, 0.1)
    #
    # for i in range(len(x3)):
    #     GRAPH.walls.add((x3[i], 11, 0))
    #     GRAPH.walls.add((x3[i], 0, 0))
    #
    # GRAPH.obstacles.add(((0, 7.1, 0), (11, 11, 0)))
    # GRAPH.obstacles.add(((0, 7.1, 0), (0, 0, 0)))

    # GRAPH, START, END = grid_from_string3("""
    #            ########
    #            #......#
    #            #......#
    #            #......#
    #            #......#
    #            #......#
    #            #......#
    #            #o.....#
    #            #......#
    #            #......#
    #            #.....X#
    #            ########
    #             """, 8, 12, 3)
    #
    # GRAPH = SquareGrid3(8, 12, 3)  # Create a Grid that is equivalent to the input grid
    # START = (3,3,0)
    # END = (5,5,0)
    #
    # y1 = np.arange(1, 7.6, 0.1)
    # y2 = np.arange(7.5, 11.1, 0.1)
    # y3 = np.arange(9, 11.1, 0.1)
    # y4 = np.arange(0, 11.1, 0.1)
    # x1 = np.arange(3, 5.3, 0.1)
    # x2 = np.arange(6, 7.1, 0.1)
    # x3 = np.arange(0, 7.1, 0.1)
    #
    # for z in np.arange(0,2.4, 0.4):
    #     for i in range(len(y1)):
    #         GRAPH.walls.add((4.4, y1[i], z))
    #     for i in range(len(y2)):
    #         GRAPH.walls.add((3.2, y2[i], z))
    #     for i in range(len(y3)):
    #         GRAPH.walls.add((1.7, y3[i], z))
    #     for i in range(len(y4)):
    #         GRAPH.walls.add((0, y4[i], z))
    #         GRAPH.walls.add((7, y4[i], z))
    #     for i in range(len(x1)):
    #         GRAPH.walls.add((x1[i], 7.5, z))
    #     for i in range(len(x2)):
    #         GRAPH.walls.add((x2[i], 7.5, z))
    #     for i in range(len(x3)):
    #         GRAPH.walls.add((x3[i], 11, z))
    #         GRAPH.walls.add((x3[i], 0, z))

    # GRAPH, START, END = grid_from_string3("""
    #            ............
    #            ............
    #            ............
    #            ............
    #            ............
    #            ............
    #            ............
    #            ............
    #            ............
    #            ............
    #            .X..........
    #            ...........o
    #             """, 12, 12, 5)
    #
    # START = (12, 12, 2)
    #
    # multiplier = 2
    # y1 = np.arange(2, 4.1, 0.1)
    # y2 = np.arange(5, 12.1, 0.1)
    # y3 = np.arange(14, 20.1, 0.1)
    # y4 = np.arange(2, 20.1, 0.1)
    # y5 = np.arange(4, 20.1, 0.1)
    # y6 = np.arange(0, 2.1, 0.1)
    # y7 = np.arange(0, 20.1, 0.1)
    # x1 = np.arange(0, 8.1, 0.1)
    # x2 = np.arange(0, 4.1, 0.1)
    # x3 = np.arange(5, 8.1, 0.1)
    # x5 = np.arange(0, 8.1, 0.1)
    # x6 = np.arange(2, 6.1, 0.1)
    # x7 = np.arange(0, 4.1, 0.1)
    # x8 = np.arange(5, 15.1, 0.1)
    # x9 = np.arange(16, 20.1, 0.1)
    # x10 = np.arange(7, 10.1, 0.1)
    # x11 = np.arange(15, 20.1, 0.1)
    # x12 = np.arange(10, 18.1, 0.1)
    # x13 = np.arange(12, 20.1, 0.1)
    # x14 = np.arange(10, 18.1, 0.1)
    # x15 = np.arange(0, 20.1, 0.1)
    #
    #
    # for z in np.arange(0, 4, 0.4):
    #
    #     for i in range(len(y1)):
    #         GRAPH.walls.add((8/multiplier, y1[i]/multiplier, z))
    #
    #     for i in range(len(y2)):
    #         GRAPH.walls.add((8/multiplier, y2[i]/multiplier, z))
    #
    #     for i in range(len(y3)):
    #         GRAPH.walls.add((6/multiplier, y3[i]/multiplier, z))
    #
    #     for i in range(len(y4)):
    #         GRAPH.walls.add((10/multiplier, y4[i]/multiplier, z))
    #
    #     for i in range(len(y5)):
    #         GRAPH.walls.add((20/multiplier, y5[i]/multiplier, z))
    #
    #     for i in range(len(y6)):
    #         GRAPH.walls.add((20/multiplier, y6[i]/multiplier, z))
    #
    #     for i in range(len(y7)):
    #         GRAPH.walls.add((0, y7[i]/multiplier,z))
    #
    #     for i in range(len(x1)):
    #         GRAPH.walls.add((x1[i]/multiplier, 4/multiplier,z))
    #
    #     for i in range(len(x2)):
    #         GRAPH.walls.add((x2[i]/multiplier, 5/multiplier,z))
    #
    #     for i in range(len(x3)):
    #         GRAPH.walls.add((x3[i]/multiplier, 5/multiplier,z))
    #
    #     for i in range(len(x5)):
    #         GRAPH.walls.add((x5[i]/multiplier, 12/multiplier,z))
    #
    #     for i in range(len(x6)):
    #         GRAPH.walls.add((x6[i]/multiplier, 14/multiplier, z))
    #
    #     for i in range(len(x7)):
    #         GRAPH.walls.add((x7[i]/multiplier, 20/multiplier, z))
    #
    #     for i in range(len(x8)):
    #         GRAPH.walls.add((x8[i]/multiplier, 20/multiplier, z))
    #
    #     for i in range(len(x9)):
    #         GRAPH.walls.add((x9[i]/multiplier, 20/multiplier, z))
    #
    #     for i in range(len(x10)):
    #         GRAPH.walls.add((x10[i]/multiplier, 14/multiplier, z))
    #
    #     for i in range(len(x11)):
    #         GRAPH.walls.add((x11[i]/multiplier, 15.9/multiplier, z))
    #
    #     for i in range(len(x12)):
    #         GRAPH.walls.add((x12[i]/multiplier, 12.1/multiplier, z))
    #
    #     for i in range(len(x13)):
    #         GRAPH.walls.add((x13[i]/multiplier, 9.4/multiplier, z))
    #
    #     for i in range(len(x14)):
    #         GRAPH.walls.add((x14[i]/multiplier, 4.5/multiplier, z))
    #
    #     for i in range(len(x15)):
    #         GRAPH.walls.add((x15[i]/multiplier, 0/multiplier, z))
    #
    #
    # for x in x15:
    #     for y in y7:
    #         z = 1.7
    #         GRAPH.walls.add((x / multiplier, y / multiplier, z))




    GRAPH, START, END = grid_from_string3("""
                ....
                ....
                ....
                .Xo.
                 """, 3, 5, 5)

    START = (1, 4, 1)
    multiplier = 2
    y1 = np.arange(0, 7.3, 0.2)
    y2 = np.arange(6.7, 7.3, 0.2)
    y3 = np.arange(0, 1.4, 0.2)
    y4 = np.arange(1.2, 4.2, 0.2)
    x1 = np.arange(0, 4, 0.2)
    x2 = np.arange(0, 0.7, 0.2)
    x3 = np.arange(0, 1.2, 0.2)
    x5 = np.arange(3.3, 4, 0.2)

    y1 = np.round(y1, 4)
    y2 = np.round(y2, 4)
    y3 = np.round(y3, 4)
    y4 = np.round(y4, 4)
    x1 = np.round(x1, 4)
    x2 = np.round(x2, 4)
    x3 = np.round(x3, 4)
    x5 = np.round(x5, 4)

    for z in np.arange(0, 2.4, 0.4):

        z = np.round(z, 4)

        for i in range(len(y1)):
            GRAPH.walls.add((0 / multiplier, y1[i] / multiplier, z))
            GRAPH.walls.add((3.8 / multiplier, y1[i] / multiplier, z))

        for i in range(len(y2)):
            GRAPH.walls.add((1 / multiplier, y2[i] / multiplier, z))

        for i in range(len(y3)):
            GRAPH.walls.add((1.4 / multiplier, y3[i] / multiplier, z))

        for i in range(len(y4)):
            GRAPH.walls.add((2.6 / multiplier, y4[i] / multiplier, z))

        for i in range(len(x1)):
            GRAPH.walls.add((x1[i] / multiplier, 0/ multiplier, z))
            GRAPH.walls.add((x1[i] / multiplier, 7.1 / multiplier, z))

        for i in range(len(x2)):
            GRAPH.walls.add((x2[i] / multiplier, 4 / multiplier, z))

        for i in range(len(x3)):
            GRAPH.walls.add((x3[i] / multiplier, 5.5 / multiplier, z))

        for i in range(len(x5)):
            GRAPH.walls.add((x5[i] / multiplier, 4 / multiplier, z))

    z = 2

    for x in x1:
        for y in np.arange(0, 1.4, 0.2):
            y = np.round(y, 4)
            GRAPH.walls.add((x / multiplier, y / multiplier, z))

    for x in np.arange(0, 2.8, 0.2):
        for y in np.arange(1, 4.2, 0.2):

            x = np.round(x, 4)
            y = np.round(y, 4)
            GRAPH.walls.add((x / multiplier, y / multiplier, z))

    for x in x1:
        for y in np.arange(4, 7.3, 0.2):

            y = np.round(y, 4)
            GRAPH.walls.add((x / multiplier, y / multiplier, z))


# # TEST
# ########################
#     z = 2
#
#     x1 = np.arange(0, 2.2*2, 0.2)
#
#
#     y1 = np.round(y1, 4)
#     y2 = np.round(y2, 4)
#     y3 = np.round(y3, 4)
#     y4 = np.round(y4, 4)
#     x1 = np.round(x1, 4)
#     x2 = np.round(x2, 4)
#     x3 = np.round(x3, 4)
#     x5 = np.round(x5, 4)
#     for x in x1:
#         for y in np.arange(0, 1.2, 0.2):
#             y = np.round(y, 4)
#             GRAPH.walls.add((x / multiplier, y / multiplier, z))
#
#     for x in np.arange(0, 2, 0.2):
#         for y in np.arange(1, 2.4, 0.2):
#
#             x = np.round(x, 4)
#             y = np.round(y, 4)
#             GRAPH.walls.add((x / multiplier, y / multiplier, z))
#
#     for x in x1:
#         for y in np.arange(2.2, 4.2, 0.2):
#
#             y = np.round(y, 4)
#             GRAPH.walls.add((x / multiplier, y / multiplier, z))
#
# ########################


    # Second floor

    y1 = np.arange(0, 7.3, 0.2)
    y2 = np.arange(0, 3.6, 0.2)
    y3 = np.arange(4.6, 5.3, 0.2)
    y4 = np.arange(5.1, 7.3, 0.2)
    x1 = np.arange(0, 4, 0.2)
    x2 = np.arange(0, 1.6, 0.2)
    x3 = np.arange(3.8, 4, 0.2)
    x5 = np.arange(0, 1.6, 0.2)

    y1 = np.round(y1, 4)
    y2 = np.round(y2, 4)
    y3 = np.round(y3, 4)
    y4 = np.round(y4, 4)
    x1 = np.round(x1, 4)
    x2 = np.round(x2, 4)
    x3 = np.round(x3, 4)
    x5 = np.round(x5, 4)

    for z in np.arange(2, 4.4, 0.4):

        z = np.round(z, 4)

        for i in range(len(y1)):
            GRAPH.walls.add((0 / multiplier, y1[i] / multiplier, z))
            GRAPH.walls.add((3.8 / multiplier, y1[i] / multiplier, z))

        for i in range(len(y2)):
            GRAPH.walls.add((2.6 / multiplier, y2[i] / multiplier, z))

        for i in range(len(y3)):
            GRAPH.walls.add((1.4 / multiplier, y3[i] / multiplier, z))

        for i in range(len(y4)):
            GRAPH.walls.add((2.6 / multiplier, y4[i] / multiplier, z))

        for i in range(len(x1)):
            GRAPH.walls.add((x1[i] / multiplier, 0/ multiplier, z))
            GRAPH.walls.add((x1[i] / multiplier, 7.1 / multiplier, z))

        for i in range(len(x2)):
            GRAPH.walls.add((x2[i] / multiplier, 5.1 / multiplier, z))

        for i in range(len(x3)):
            GRAPH.walls.add((x3[i] / multiplier, 5.1 / multiplier, z))

        for i in range(len(x5)):
            GRAPH.walls.add((x5[i] / multiplier, 3.4 / multiplier, z))

    z = 4
    for x in x1:
        for y in y1:
            GRAPH.walls.add((x / multiplier, y / multiplier, z))

    start = time.time()
    dstar = adaptive3(GRAPH, START, END)  # Define an instance of the DStarLite class using the graph object and
    # the start and end positions

    ax, ax_label = dstar.initialise()
    count = 1
    while dstar.position != dstar.start or len(dstar.seen) != len(dstar.graph.available_nodes):
        # while the goal has not been reached
        waypoint = dstar.move_to_goal(ax, ax_label)

        # print(waypoint)
        if count > 5 and dstar.position == dstar.start:
            print(len(dstar.graph.available_nodes))
            print(len(dstar.seen))
            break

        count += 1
    print("time {}".format(time.time() - start))
