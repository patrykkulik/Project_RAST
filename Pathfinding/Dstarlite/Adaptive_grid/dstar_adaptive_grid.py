from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from functools import partial


# Import needed functions from other scripts:
from utility_test import draw_grid
from utility_test import label_grid
from utility_test import grid_from_string
from priority_queue import PriorityQueue
from adaptive_grid import AgentViewGrid, SquareGrid


def distance(a, b):
    """ Calculate a straight line distance between points a and b on a 2D plane """
    (x1, y1) = a
    (x2, y2) = b
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def heuristic(a, b):
    """ Calculate the heuristic between points a and b on a 2D plane """
    # Returns a min distance between two nodes assuming motion is only possible at 0, 45 and 90 degree angles
    (x1, y1) = a
    (x2, y2) = b
    diagonal_distance = min(abs(x1 - x2), abs(y1 - y2))
    straight_distance = max(abs(x1 - x2), abs(y1 - y2)) - diagonal_distance
    return diagonal_distance*np.sqrt(2) + straight_distance

#######################################################################################################################


class FieldDStarLite(object):
    def __init__(self, graph, start, goal, view_range=1.9):
        # Initialise the Field D* algorithm:

        self.graph = AgentViewGrid(graph.width, graph.height)  # Define the graph method as an instance of AgentView
        # (the map from the perspective of the robot)
        self.real_graph: SquareGrid = graph  # Make the real_graph method equal to the real version of the grid
        self.view_range = view_range  # Define the range of view of the robot

        self.seen = set()  # Create a set of all the seen nodes

        self.Km = 0  # Key modifier. When the robot makes a move, this is incremented so that heuristic? can stay same
        self.real_world_multiplier = 3  # how many meters is one step equivalent to in the real world
        self.position = start  # Starting position of the robot
        self.start = start
        self.goal = (500, 500)  # Goal position (fake)
        # Create dictionaries:
        self.G_VALS = {start: 1000,
                       self.goal: float('inf')}
        self.RHS_VALS = {start: 1000, self.goal: 0}  # Dictionary of all rhs values for all of the expanded cells

        self.frontier = PriorityQueue()  # Define the frontier method as an instance of PriorityQueue
        self.frontier.put(self.goal, self.calculate_key(self.goal))  # Append the goal and its key to the frontier

    def initialise_plot(self):
        plt.figure()
        plt.grid()
        ax = plt.subplot(1, 1, 1)
        # ax_label = plt.subplot(1, 2, 2)
        ax_label = 1

        ax.set_ylim(-0.5, self.real_world_multiplier*max(self.graph.height, self.graph.width) - self.real_world_multiplier/2)
        ax.set_xlim(-0.5, self.real_world_multiplier*max(self.graph.height, self.graph.width) - self.real_world_multiplier/2)
        ax.xaxis.set_major_locator(plt.MultipleLocator(self.real_world_multiplier))
        ax.yaxis.set_major_locator(plt.MultipleLocator(self.real_world_multiplier))
        plt.pause(3)

        return ax, ax_label

    def rhs(self, node):
        """
        Return node's rhs value. If node not in the RHS_VALS dict, return inf. If node is the goal node, return 0
        """
        # rhs value is what the g value of the node should be
        if node == self.goal:
            return 0  # The goal node should always be zero
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
        if self.obstacle_check(node):
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
            np.round(g_rhs + heuristic(node, self.position) + self.Km, 4),
            np.round(g_rhs, 4)
            )
        # The key is a combination of the total function and the start distance. A rounding is applied to prevent
        # machine precision from introducing problems

    def obstacle_check(self, node):
        """ Check if the node is inside an obstacle """
        if node in self.graph.openings:
            False
        # If coordinate in available coordinates, check the vertex
        elif node[0] % 0.125 == 0 and node[1] % 0.125 == 0 and node in self.graph.available_nodes:
            if node in self.graph.walls:
                return True
            else:
                return False
        #     # Otherwise, check the verticies at the ends of the current edge
        # elif node[0] % 1 != 0:
        #     if (np.floor(node[0]), node[1]) in self.graph.walls and (np.ceil(node[0]), node[1]) in self.graph.walls:
        #         return True
        #     else:
        #         return False
        # elif node[1] % 1 != 0:
        #     if (node[0], np.floor(node[1])) in self.graph.walls and (node[0], np.ceil(node[1])) in self.graph.walls:
        #         return True
        #     else:
        #         return False
        else:
            return False

    def feasible_motion(self, from_node, to_node):
        """ Function to check if motion between two nodes is feasible (no obstacle present in between) """
        # Could be improved to make it less convoluted
        return not self.obstacle_check(to_node)
        # if from_node[0] % 0.5 != 0 and to_node[0] % 0.5 != 0 and to_node[1] != from_node[1]:
        #     px = np.round((from_node[0] + to_node[0])/2)
        #     py = (from_node[1] + to_node[1])/2
        #     return not self.obstacle_check((px, py))
        # elif from_node[1] % 0.5 != 0 and to_node[1] % 0.5 != 0 and to_node[0] != from_node[0]:
        #     py = np.round((from_node[1] + to_node[1])/2)
        #     px = (from_node[0] + to_node[0])/2
        #     return not self.obstacle_check((px, py))
        # elif from_node[0] % 0.5 != 0:
        #     if abs(to_node[0] - from_node[0]) > 1:
        #         py = (from_node[1] + to_node[1]) / 2
        #         px = np.round(from_node[0])
        #         return not self.obstacle_check((px, py))
        #     elif abs(to_node[1] - from_node[1]) > 1:
        #         px = (from_node[0] + to_node[0]) / 2
        #         py = np.round(from_node[1])
        #         return not self.obstacle_check((px, py))
        #     else:
        #         return True
        # elif from_node[1] % 0.5 != 0:
        #     if abs(to_node[1] - from_node[1]) > 1:
        #         px = (from_node[0] + to_node[0]) / 2
        #         py = np.round(from_node[1])
        #         return not self.obstacle_check((px, py))
        #     elif abs(to_node[0] - from_node[0]) > 1:
        #         px = (from_node[1] + to_node[1]) / 2
        #         py = np.round(from_node[0])
        #         return not self.obstacle_check((px, py))
        #     else:
        #         return True
        # else:
        #     py = (from_node[1] + to_node[1]) / 2
        #     px = (from_node[0] + to_node[0]) / 2
        #     return (not self.obstacle_check((px, py)) and not self.obstacle_check((to_node[0], to_node[1])))

        # else:
        #     return True

    def lookahead_cost(self, node, neighbour):
        c = 2
        return self.g(neighbour) + c*distance(neighbour, node)

    def compute_cost(self, node, neighbors):
        """Compute cost function developed by Dave Ferguson and Anthony Stentz
        (https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_4/ferguson_david_2005_4.pdf)
        :return vs: rhs of node s from edge sa <-> sb
         destination: the destination to obtain optimal cost
        """

        cost = partial(self.lookahead_cost, node)
        destination = min(neighbors, key=cost)  # Find the point closest to id in the available nodes set

        while not self.feasible_motion(node, destination) and len(neighbors) > 0:
            destination = min(neighbors, key=cost)
            neighbors.remove(destination)

        if len(neighbors) != 0:
            vs = self.lookahead_cost(node, destination)
        else:
            vs = float('inf')

        return vs, destination

    def calculate_rhs(self, node):
        """ Calculate the rhs value of node """
        if self.obstacle_check(node):  # If node is in an obstacle, rhs is equal to inf
            return float('inf')
        elif node not in self.graph.available_nodes:
            return float('inf')
        else:
            # Compute all local neighbors of the node
            local_neighbors = self.graph.neighbors(node)


            vs, destination = self.compute_cost(node, local_neighbors)  # compute the cost

            # for opening in self.graph.openings:
            #     if node[0] - 1.5 <= opening[0] <= node[0] + 1.5 and node[1] - 1.5 <= opening[1] <= \
            #             node[1] + 1.5 and self.feasible_motion(node, opening):
            #         vs_temp = self.g(opening) + 2 * distance(opening, node)
            #         destination = opening
            #         if vs_temp < vs:
            #             vs = vs_temp
            #             self.back_pointers[node] = destination
            return vs

    def update_node(self, node):
        """ Update the rhs value of a node and put in on the frontier if it is inconsistent """

        if node != self.goal and node in self.seen and node != self.start:
            self.RHS_VALS[node] = self.calculate_rhs(node)
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
                    self.update_nodes(self.graph.neighbors((2, 2)))

                # for opening in self.graph.openings:
                #     if node[0] - 1.5 <= opening[0] <= node[0] + 1.5 and node[1] - 1.5 <= opening[1] <= \
                #                     node[1] + 1.5:
                #         self.update_node(opening)

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
                    self.update_nodes(self.graph.neighbors((2, 2)))

            if self.frontier.empty():
                break

        return self.G_VALS.copy()

    def adapt_grid(self, new_walls, ax, robot_plot=True):
        new_nodes = []
        for wall in new_walls:
            wall = tuple(np.round(wall, 4))
            ax.plot(self.real_world_multiplier * wall[0], self.real_world_multiplier * wall[1], 'ko')

            if wall not in self.graph.available_nodes:
                pos = (np.floor(wall[0] * 8) / 8 + 1/16, np.floor(wall[1] * 8) / 8 + 1/16)  # position the wall node in the middle

                for delx in [-1 / 16, 1 / 16]:
                    for dely in [-1 / 16, 1 / 16]:
                        # if delx == 0 and dely == 0:
                        #     continue
                        new_node = (pos[0] + delx, pos[1] + dely)
                        if self.graph.in_bounds(new_node):
                            self.graph.available_nodes.update([new_node])
                            self.graph.walls.update([new_node])
                            self.update_node(new_node)
                            if robot_plot == True:
                                ax.plot(self.real_world_multiplier * new_node[0], self.real_world_multiplier * new_node[1],
                                        'o', color='gray')

                            # ax.plot(self.real_world_multiplier * new_node[0], self.real_world_multiplier * new_node[1], 'o', color='gray', alpha=0.1)
                            self.seen.update([new_node])
                            new_nodes.append(new_node)

        for new_node in new_nodes:

            for neighbor in self.graph.neighbors(new_node):
                if distance(new_node, neighbor) >= 0.25:
                    if neighbor in self.graph.walls:
                        px = (neighbor[0] + new_node[0])/2
                        py = (neighbor[1] + new_node[1])/2
                        con_x = False
                        con_y = False

                        if px % 0.125 == 0 and py % 0.125 == 0:
                            open_node = (px, py)
                            self.graph.available_nodes.update([open_node])
                            self.update_node(open_node)
                            ax.plot(self.real_world_multiplier * px, self.real_world_multiplier * py, 'o', color='gray', alpha=0.1)
                            self.seen.update([open_node])
                        elif px % 0.125 == 0:
                            for dy in [-1/16, 1/16]:
                                open_node = (px, py + dy)
                                if py +dy == new_node[1] or py + dy == neighbor[1]:
                                    con_y = True

                                self.graph.available_nodes.update([open_node])
                                self.update_node(open_node)
                                ax.plot(self.real_world_multiplier * open_node[0], self.real_world_multiplier * open_node[1], 'o', color='gray', alpha=0.1)
                                self.seen.update([open_node])
                        elif py % 0.125 == 0:
                            for dx in [-1 / 16, 1 / 16]:
                                open_node = (px + dx, py)
                                if px +dx == new_node[0] or px + dx == neighbor[0]:
                                    con_x = True

                                self.graph.available_nodes.update([open_node])
                                self.update_node(open_node)
                                ax.plot(self.real_world_multiplier * open_node[0],
                                        self.real_world_multiplier * open_node[1], 'o', color='gray',
                                        alpha=0.1)
                                self.seen.update([open_node])
                        else:
                            for dx in [-1 / 16, 1 / 16]:
                                for dy in [-1 / 16, 1 / 16]:
                                    open_node = (px + dx, py + dy)
                                    if py + dy == new_node[1] or py + dy == neighbor[1]:
                                        con_y = True
                                    elif px +dx == new_node[0] or px + dx == neighbor[0]:
                                        con_x = True

                                    self.graph.available_nodes.update([open_node])
                                    self.update_node(open_node)
                                    ax.plot(self.real_world_multiplier * open_node[0],
                                            self.real_world_multiplier * open_node[1], 'o',
                                            color='gray',
                                            alpha=0.1)
                                    self.seen.update([open_node])
                        # if neighbor[0] == new_node[0]:
                        if con_x:
                            open_node_1 = (np.ceil(px), np.floor(py*8)/8)
                            open_node_2 = (np.floor(px), np.floor(py*8)/8)

                            self.graph.available_nodes.update([open_node_1])
                            self.update_node(open_node_1)
                            ax.plot(self.real_world_multiplier * open_node_1[0],
                                    self.real_world_multiplier * open_node_1[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_1])

                            self.graph.available_nodes.update([open_node_2])
                            self.update_node(open_node_2)
                            ax.plot(self.real_world_multiplier * open_node_2[0],
                                    self.real_world_multiplier * open_node_2[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_2])



                            open_node_3 = (np.ceil(px)+1, np.floor(py * 8) / 8)
                            open_node_4 = (np.floor(px)-1, np.floor(py * 8) / 8)

                            self.graph.available_nodes.update([open_node_3])
                            self.update_node(open_node_3)
                            ax.plot(self.real_world_multiplier * open_node_3[0],
                                    self.real_world_multiplier * open_node_3[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_3])

                            self.graph.available_nodes.update([open_node_4])
                            self.update_node(open_node_4)
                            ax.plot(self.real_world_multiplier * open_node_4[0],
                                    self.real_world_multiplier * open_node_4[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_4])

                        # elif neighbor[1] == new_node[1]:
                        elif con_y:
                            open_node_1 = (np.floor(px*8)/8, np.ceil(py))
                            open_node_2 = (np.floor(px*8)/8, np.floor(py))

                            self.graph.available_nodes.update([open_node_1])
                            self.update_node(open_node_1)
                            ax.plot(self.real_world_multiplier * open_node_1[0],
                                    self.real_world_multiplier * open_node_1[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_1])

                            self.graph.available_nodes.update([open_node_2])
                            self.update_node(open_node_2)
                            ax.plot(self.real_world_multiplier * open_node_2[0],
                                    self.real_world_multiplier * open_node_2[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_2])




                            open_node_3= (np.floor(px * 8) / 8, np.ceil(py)+1)
                            open_node_4 = (np.floor(px * 8) / 8, np.floor(py)-1)

                            self.graph.available_nodes.update([open_node_3])
                            self.update_node(open_node_3)
                            ax.plot(self.real_world_multiplier * open_node_3[0],
                                    self.real_world_multiplier * open_node_3[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_3])

                            self.graph.available_nodes.update([open_node_4])
                            self.update_node(open_node_4)
                            ax.plot(self.real_world_multiplier * open_node_4[0],
                                    self.real_world_multiplier * open_node_4[1], 'o', color='gray',
                                    alpha=0.1)
                            self.seen.update([open_node_4])



                # if wall not in self.graph.available_nodes:
            #     pos = (np.floor(wall[0]) + 0.5, np.floor(wall[1]) + 0.5)

                # for delx in [-0.5, 0, 0.5]:
                #     for dely in [-0.5, 0, 0.5]:
                #         new_node = (pos[0] + delx, pos[1] + dely)
                #         if new_node not in self.graph.available_nodes and self.graph.in_bounds(new_node):
                #             self.graph.available_nodes.update([new_node])
                #             self.update_node(new_node)
                #             ax.plot(self.real_world_multiplier * new_node[0], self.real_world_multiplier * new_node[1], 'o', color='gray', alpha=0.1)
                #             self.seen.update([new_node])
                #
                # pos = (np.floor(wall[0] * 2) / 2 + 0.25, np.floor(wall[1] * 2) / 2 + 0.25)
                #
                # for delx in [-0.25, 0, 0.25]:
                #     for dely in [-0.25, 0, 0.25]:
                #         new_node = (pos[0] + delx, pos[1] + dely)
                #         if new_node not in self.graph.available_nodes and self.graph.in_bounds(new_node):
                #             self.graph.available_nodes.update([new_node])
                #             self.update_node(new_node)
                #             ax.plot(self.real_world_multiplier * new_node[0], self.real_world_multiplier * new_node[1], 'o', color='gray', alpha=0.1)
                #             self.seen.update([new_node])
                #
                # pos = (np.floor(wall[0] * 4) / 4 + 0.25 / 2, np.floor(wall[1] * 4) / 4 + 0.25 / 2)
                #
                # for delx in [-1 / 8, 0, 1 / 8]:
                #     for dely in [-1 / 8, 0, 1 / 8]:
                #         new_node = (pos[0] + delx, pos[1] + dely)
                #         if new_node not in self.graph.available_nodes and self.graph.in_bounds(new_node):
                #             self.graph.available_nodes.update([new_node])
                #             self.update_node(new_node)
                #             ax.plot(self.real_world_multiplier * new_node[0], self.real_world_multiplier * new_node[1], 'o', color='gray', alpha=0.1)
                #             self.seen.update([new_node])
                #
                # pos = (np.floor(wall[0] * 8) / 8, np.floor(wall[1] * 8) / 8)
                #
                # for delx in [0, 1 / 8]:
                #     for dely in [0, 1 / 8]:
                #         new_node = (pos[0] + delx, pos[1] + dely)
                #         if self.graph.in_bounds(new_node):
                #             self.graph.walls.update([new_node])
                #             self.update_node(new_node)
                #
                #             # self.update_nodes([node for node in self.graph.neighbors((pos[0] + delx, pos[1] + dely))
                #             #                        if node not in self.graph.walls])
                #
                #             if robot_plot == True:
                #                 ax.plot(self.real_world_multiplier * new_node[0], self.real_world_multiplier * new_node[1], 'o', color='gray')
                #             self.seen.update([new_node])


    def move_to_goal(self):
        # Initialise the plot axes and draw the grid
        ax, ax_label = self.initialise_plot()

        observation = self.real_graph.observe(self.position, self.view_range)  # Observe the surroundings in the view
        # range
        walls = self.graph.new_walls(observation)  # Define the position of newly observed walls to the graph seen by
        # the robot
        openings = self.graph.new_openings(observation)

        self.update_nodes(openings)
        self.graph.openings.update(openings)
        self.graph.available_nodes.update(openings)
        self.graph.update_walls(walls)  # Update the walls on the robot's map

        self.adapt_grid(walls, ax, robot_plot=True)

        for n in list(observation.keys()):
            if n in self.graph.available_nodes:
                self.seen.update([n])

        # Compute the shortest path to the goal based on the current information
        self.compute_shortest_path()
        last_node = self.position  # Define last node as the current position

        yield self.position, observation, self.graph.walls

        # label_grid(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal, self.real_world_multiplier)
        draw_grid(self.graph, ax, self.view_range,  self.real_world_multiplier, path=self.position, goal=self.goal)

        while self.position != self.start or len(self.seen) != len(self.graph.available_nodes):  # while the goal has not been reached
            condition = True

            # If the robot becomes stuck in a position with inf g, there is no path that can be taken
            if self.rhs(self.position) == float('inf'):
                raise Exception("No path")

            best_step = self.position

            # for opening in self.graph.openings:
            #     if self.position[0]-1 <= opening[0] <= self.position[0]+1 and self.position[1] - 1 <= opening[1] <= \
            #             self.position[1] + 1:
            #         neighbors.append(opening)

            neighbors = self.graph.neighbors(self.position)
            # best_step = neighbors[0]

            for neighbor in neighbors:
                if self.g(neighbor) + distance(neighbor, self.position)*2 <= self.g(best_step) + distance(best_step, self.position)*2:
                    best_step = neighbor

            self.position = best_step

            observation = self.real_graph.observe(self.position, self.view_range)  # make a new observation
            new_walls = self.graph.new_walls(observation)  # obtain the new walls from the observation
            openings = self.graph.new_openings(observation)

            newly_seen = []
            for n in list(observation.keys()):
                if n in self.graph.available_nodes:
                    if n not in self.seen:
                        newly_seen.append(n)
                    self.seen.update([n])


            # If new walls have been observed, update walls, the key modifier and nodes
            if new_walls or openings:
                self.graph.available_nodes.update(openings)

                for o in openings:
                    ax.plot(self.real_world_multiplier * o[0], self.real_world_multiplier * o[1], 'o', color = 'green')

                self.update_nodes(openings)
                self.graph.openings.update(openings)

                self.graph.update_walls(new_walls)
                # self.update_node(self.position)
                self.Km += heuristic(last_node, self.position)  # updates the key modifier by how much the robot has
                # moved since the last time it saw a wall
                last_node = self.position

                self.adapt_grid(new_walls, ax, robot_plot=True)

                self.update_node(self.position)
                self.update_node(self.start)
                self.compute_shortest_path()  # Recompute the shortest path

            elif len(self.seen) == len(self.graph.available_nodes): # If all points have been seen
                if self.position == self.start:
                    print(len(self.graph.available_nodes))
                    print(self.graph.height)
                    print(self.graph.width)
                    pass
                else:

                    self.update_node(self.position)
                    self.update_node(self.start)
                    self.update_node((self.graph.width - 2, self.graph.height - 2))
                    self.update_node((2, 2))
                    self.update_node((2, self.graph.height - 2))
                    self.update_node((self.graph.width - 2, 2))
                    self.update_nodes(self.seen)
                    self.compute_shortest_path()  # Recompute the shortest path

            else:
                self.update_node(self.position)
                self.G_VALS[self.position] = self.rhs(self.position)
                for n in self.graph.neighbors(self.position):
                    self.update_node(n)
                    self.G_VALS[n] = self.rhs(n)

                self.update_node(self.position)
                self.G_VALS[self.position] = self.rhs(self.position)

                if len(newly_seen) != 0:
                    keys = list(self.RHS_VALS.keys())
                    for key in keys:
                        if \
                                self.position[0] - (self.view_range+1) <= key[0] <= self.position[0] + self.view_range+1 and self.position[1] - (self.view_range+1) <= key[1] <= \
                                        self.position[1] + self.view_range+1:
                            if self.RHS_VALS[key] == 1:
                                condition = False
                                break

                # Condition only True if the robot does not have unvisited nodes just outside its field of vision
                if condition:
                    self.update_node(self.start)
                    self.compute_shortest_path()  # Recompute the shortest path


            yield self.position, observation, self.graph.walls
            # label_grid(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal, self.real_world_multiplier)
            draw_grid(self.graph, ax, self.view_range, self.real_world_multiplier, path=self.position, goal=self.goal)


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

    GRAPH, START, END = grid_from_string("""
               ########
               #....#.#
               #....#.#
               #....#.#
               #......#
               #......#
               #......#
               #o.....#
               #......#
               #......#
               #.....X#
               ########
                """)

    y1 = np.arange(1, 7.6, 0.1)

    for i in range(len(y1)):
        GRAPH.walls.add((4.4, y1[i]))

    GRAPH.obstacles.add(((4.4, 4.4), (1, 7.5)))

    y2 = np.arange(7.5, 11.1, 0.1)

    for i in range(len(y2)):
        GRAPH.walls.add((3.2, y2[i]))

    GRAPH.obstacles.add(((3.2, 3.2), (7.5, 11)))

    y3 = np.arange(9, 11.1, 0.1)

    for i in range(len(y3)):
        GRAPH.walls.add((1.7, y3[i]))

    GRAPH.obstacles.add(((1.7, 1.7), (9, 11)))

    y4 = np.arange(0, 11.1, 0.1)

    for i in range(len(y4)):
        GRAPH.walls.add((0, y4[i]))
        GRAPH.walls.add((7, y4[i]))

    GRAPH.obstacles.add(((0, 0), (0, 11)))
    GRAPH.obstacles.add(((7, 7), (0, 11)))


    x1 = np.arange(3, 5.3, 0.1)

    for i in range(len(x1)):
        GRAPH.walls.add((x1[i], 7.5))

    GRAPH.obstacles.add(((3, 5.2), (7.5, 7.5)))


    x2 = np.arange(6, 7.1, 0.1)

    for i in range(len(x2)):
        GRAPH.walls.add((x2[i], 7.5))

    GRAPH.obstacles.add(((6, 7.1), (7.5, 7.5)))


    x3 = np.arange(0, 7.1, 0.1)

    for i in range(len(x3)):
        GRAPH.walls.add((x3[i], 11))
        GRAPH.walls.add((x3[i], 0))






    GRAPH, START, END = grid_from_string("""
               ........
               ........
               ........
               ........
               ........
               ........
               ........
               X......o
                """)

    y1 = np.arange(2, 4.1, 0.1)

    for i in range(len(y1)):
        GRAPH.walls.add((8, y1[i]))

    y2 = np.arange(5, 12.1, 0.1)

    for i in range(len(y2)):
        GRAPH.walls.add((8, y2[i]))

    y3 = np.arange(14, 20.1, 0.1)

    for i in range(len(y3)):
        GRAPH.walls.add((5, y3[i]))

    y4 = np.arange(2, 20.1, 0.1)

    for i in range(len(y4)):
        GRAPH.walls.add((10, y4[i]))

    y5 = np.arange(2, 20.1, 0.1)

    for i in range(len(y4)):
        GRAPH.walls.add((10, y4[i]))



    x1 = np.arange(3, 5.3, 0.1)

    for i in range(len(x1)):
        GRAPH.walls.add((x1[i], 7.5))

    GRAPH.obstacles.add(((3, 5.2), (7.5, 7.5)))


    x2 = np.arange(6, 7.1, 0.1)

    for i in range(len(x2)):
        GRAPH.walls.add((x2[i], 7.5))

    GRAPH.obstacles.add(((6, 7.1), (7.5, 7.5)))


    x3 = np.arange(0, 7.1, 0.1)

    for i in range(len(x3)):
        GRAPH.walls.add((x3[i], 11))
        GRAPH.walls.add((x3[i], 0))




    # GRAPH.openings.add((5.25, 7.5))
    # GRAPH.openings.add((5.25, 7))
    # GRAPH.openings.add((5.25, 8))


    dstar = FieldDStarLite(GRAPH, START, END)  # Define an instance of the DStarLite class using the graph object and
    # the start and end positions

    path = [p for p, o, w in dstar.move_to_goal()]
    plt.show()

