#
# greedy3D.py
#
# Created on 28/05/2020
#
#

from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from functools import partial
from mpl_toolkits.mplot3d import Axes3D

# Import needed functions from other scripts:
from utility_field3 import draw_grid3
from utility_field3 import label_grid3
from utility_field3 import grid_from_string3
from priority_queue import PriorityQueue
from grid_field3 import AgentViewGrid3, SquareGrid3

def distance3(a, b):
    """ Calculate a straight line distance between points a and b on a 2D plane """

    (x1, y1, z1) = a
    (x2, y2, z2) = b

    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def heuristic3(a, b):
    """ Calculate the heuristic between points a and b on a 2D plane """
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

class greedyDStarLite3(object):
    def __init__(self, graph, start, goal, view_range=1.9):
        # Initialise the Field D* algorithm:

        self.graph = AgentViewGrid3(graph.length, graph.width,
                                    graph.height)  # Define the graph method as an instance of AgentView
  
        # (the map from the perspective of the robot)
        self.real_graph: SquareGrid3 = graph  # Make the real_graph method equal to the real version of the grid
        self.view_range = view_range  # Define the range of view of the robot

        self.seen = set()  # Create a set of all the seen nodes

        # Create dictionaries:
        self.real_world_multiplier = 1
        self.back_pointers = {}  # Dictionary describing the first guess for the path of least resistance
        # self.G_VALS = {start: 1000,
        #                goal: float('inf')}  # Dictionary of all g values for all of the expanded cells
        self.Km = 0  # Key modifier. When the robot makes a move, this is incremented so that heuristic can stay same
        self.position = start  # Starting position of the robot
        self.start = start
        self.goal = (500, 500, 500)  # Goal position (fake)
        self.G_VALS = {start: 1000,
                       self.goal: float('inf')}
        self.RHS_VALS = {start: 1000, self.goal: 0}  # Dictionary of all rhs values for all of the expanded cells

        self.frontier = PriorityQueue()  # Define the frontier method as an instance of PriorityQueue
        self.frontier.put(self.goal, self.calculate_key(self.goal))  # Append the goal and its key to the frontier

    def initialise_plot(self):
        fig = plt.figure()
        
        # plt.grid()
        ax = fig.add_subplot(1, 1, 1, projection='3d')  # axis showing the movement of the robot
        # ax.grid()
        # ax_label = fig.add_subplot(1, 2, 2, projection='3d')  # axis showing the labels of the nodes

        ax_label = 1

        # Set the appropriate axes limits
        ax.set_xlim(0, self.graph.length)
        ax.set_ylim(0, self.graph.width)
        ax.set_zlim(0, self.graph.height)

        # set a tick increment to 1 for all axes:
        ax.xaxis.set_major_locator(plt.MultipleLocator(1))
        ax.yaxis.set_major_locator(plt.MultipleLocator(1))
        ax.zaxis.set_major_locator(plt.MultipleLocator(1))
        plt.pause(1)

Adaptive-grid
        for a in self.graph.available_nodes:
            ax.plot([self.real_world_multiplier * a[0]], [self.real_world_multiplier * a[1]],
                    [self.real_world_multiplier * a[2]], 'o', color='gray', alpha=0.1)

        ax.view_init(elev=10, azim=290)


        return ax, ax_label

    def rhs(self, node):
        """ Return node's rhs value. If node not in the RHS_VALS dict, return inf. If node is the goal node, return 0 """
        # rhs value is what the g value of the node should be

        if node == self.goal:

            return 0  # The goal node should always be zero


        elif node == self.start:
            return 1000  # To ensure that the drone comes back to start at the end, the start node should have a constant, high rhs

        elif node == self.position:
            return 1.2 * self.RHS_VALS.get(node, 1)

        else:
            return self.RHS_VALS.get(node, 1)

        # return self.RHS_VALS.get(node, float('inf')) if node != self.goal else 0

    def g(self, node):
        """ Return the node's g value. If node not present in the G_VALS dict, or node is an obstacle, return inf """
        # g value is essentially the optimal distance from the current node to the goal node (given the current knowledge of the map)

        if self.obstacle_check3(node):
            return_val = float('inf')
        else:
            return_val = self.G_VALS.get(node, float('inf'))

        return return_val

    def calculate_key(self, node):
        """ Return the key pair for a given node """
        # Key determines the priority of a node on the expansion queue. The lower the key, the closer the node is to the goal. On the priority queue, nodes are expanded and updated according to the priority key

        g_rhs = min([self.g(node), self.rhs(node)])  # Since g and rhs should be equal for an expanded node, a min value
        # is taken to ensure that node is not expanded too late

        return (
            np.round(g_rhs + heuristic3(node, self.position) + self.Km, 4),
            np.round(g_rhs, 4)
        )
        # The key is a combination of the total function and the start distance. A rounding is applied to prevent machine precision from introducing problems

    def obstacle_check3(self, node):
        """ Check if the node is inside an obstacle """
        if node[0] % 0.125 == 0 and node[1] % 0.125 == 0 and node[2] % 0.125 == 0 and node in self.graph.available_nodes:

            if node in self.graph.walls:
                return True
            else:
                return False

        # # If all coordinates are integers, check the vertex
        # if node[0] % 1 == 0 and node[1] % 1 == 0 and node[2] % 1 == 0:
        #     if node in self.graph.walls:
        #         return True
        #     else:
        #         return False
        #
        # # Otherwise, check the vertices at the corners of the surface that contains it
        # elif node[0] % 1 == 0 and node[1] % 1 != 0 and node[2] % 1 != 0:
        #     if (node[0], np.floor(node[1], np.floor(node[2]))) in self.graph.walls and (
        #     node[0], np.floor(node[1], np.ceil(node[2]))) in self.graph.walls and (
        #     node[0], np.ceil(node[1], np.floor(node[2]))) in self.graph.walls and (
        #     node[0], np.ceil(node[1], np.ceil(node[2]))) in self.graph.walls:
        #         return True
        #
        #     else:
        #         return False
        #
        # elif node[1] % 1 == 0 and node[0] % 1 != 0 and node[2] % 1 != 0:
        #     if (node[1], np.floor(node[0], np.floor(node[2]))) in self.graph.walls and (
        #     node[1], np.floor(node[0], np.ceil(node[2]))) in self.graph.walls and (
        #     node[1], np.ceil(node[0], np.floor(node[2]))) in self.graph.walls and (
        #     node[1], np.ceil(node[0], np.ceil(node[2]))) in self.graph.walls:
        #         return True
        #
        #     else:
        #         return False
        #
        #
        # elif node[2] % 1 == 0 and node[0] % 1 != 0 and node[1] % 1 != 0:
        #     if (node[2], np.floor(node[0], np.floor(node[1]))) in self.graph.walls and (
        #     node[2], np.floor(node[0], np.ceil(node[1]))) in self.graph.walls and (
        #     node[2], np.ceil(node[0], np.floor(node[1]))) in self.graph.walls and (
        #     node[2], np.ceil(node[0], np.ceil(node[1]))) in self.graph.walls:
        #         return True
        #
        #     else:
        #         return False
        #
        # else:
        #     return False

    def feasible_motion3(self, from_node, to_node):
        """ Function to check if motion between two nodes is feasible (no obstacle present in between) """
        # modified to check only vertex

        return not (self.obstacle_check3(to_node) or self.obstacle_check3(from_node))

    def lookahead_cost(self, node, neighbour):
        c = 2
        return self.g(neighbour) + c * distance3(neighbour, node)

    # def compute_cost(self, node, neighbours):
    #     """Compute cost function developed by Dave Ferguson and Anthony Stentz
    #     (https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_4/ferguson_david_2005_4.pdf)
    #     :return vs: rhs of node s from edge sa <-> sb
    #      destination: the destination to obtain optimal cost
    #     """
    #     vs = float('inf')
    #     destination = None
    #     for s2 in neighbours:
    #         s1 = node
    #         if self.feasible_motion3(s1,s2):
    #             if self.lookahead_cost(s1, s2) < vs:
    #                 vs = self.lookahead_cost(s1, s2)
    #                 destination = s1
    #
    #     return vs, destination

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
            vs, destination = self.compute_cost(node, local_neighbors)  # compute the cost
            return vs

    def update_node(self, node):
        """ Update the rhs value of a node and put in on the frontier if it is inconsistent """

        if node != self.goal and node in self.seen and node != self.start:
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

            # If the g(node) > rhs(node) we have a simple situation where we can just set g(node) = rhs(node) and update neighbors
            elif self.g(node) > self.rhs(node):
                self.G_VALS[node] = self.rhs(node)
                if node != self.goal:
                    self.update_nodes(self.graph.neighbors(node))
                    if node == self.position:
                        self.update_node(node)
                else:
                    self.update_nodes(self.graph.neighbors((4, 2, 0)))
                    self.update_nodes(self.graph.neighbors((4, 4, 0)))
            # If the g(node) < rhs(node) we have a difficult situation where we first have to set the g(node) = inf,
            # after which we move the node and its neighbors back to the frontier. This ensures that the node is not
            # updated more than necessary. After the required nodes move through the queue and this node is being
            # expanded, the situation becomes a simple case where the g(node) is relaxed to its rhs value

            else:
                self.G_VALS[node] = float('inf')
                if node != self.goal:
                    for n in self.graph.neighbors(node):
                        self.update_node(n)

                    self.update_node(node)


                else:
                    self.update_node(node)
                    self.update_nodes(self.graph.neighbors((4, 2, 0)))

            if self.frontier.empty():
                break

        return self.G_VALS.copy()

    def split_grid(self, wall, split, ax):
        dnom = 2 ** split
        frac = 1 / (2 ** split)

        pos = (np.floor(wall[0] * (2 ** (split - 1))) / (2 ** (split - 1)) + frac,
               np.floor(wall[1] * (2 ** (split - 1))) / (2 ** (split - 1)) + frac,
               np.floor(wall[2] * 2) / (2)) # Approximate to nearest half

        for delx in [-frac, 0, frac]:
            for dely in [-frac, 0, frac]:
                for delz in [-0.5, 0, 0.5]:
                    new_node = (pos[0] + delx, pos[1] + dely, pos[2] + delz)
                    if new_node not in self.graph.available_nodes and self.graph.in_bounds(new_node):
                        self.graph.available_nodes.update([new_node])
                        self.update_node(new_node)
                        ax.plot([self.real_world_multiplier * new_node[0]], [self.real_world_multiplier * new_node[1]],
                                [self.real_world_multiplier * new_node[2]], 'o', color='gray', alpha=0.1)
                        self.seen.update([new_node])

    def adapt_grid(self, new_walls, ax, robot_plot=True):

        for wall in new_walls:
            wall = tuple(np.round(wall, 4))
            ax.plot([self.real_world_multiplier * wall[0]], [self.real_world_multiplier * wall[1]], [self.real_world_multiplier * wall[2]], 's', color = 'black')

            if wall not in self.graph.available_nodes:
                splits = 2
                for split in range(1,splits+1):
                    self.split_grid(wall, split, ax)

                # finishing
                dnom = 2 ** splits
                frac = 1 / (2 ** splits)

                pos = (
                np.floor(wall[0] * dnom) / dnom, np.floor(wall[1] * dnom) / dnom, np.floor(wall[2] * 2) / (2))

                for delx in [0, frac]:
                    for dely in [0, frac]:
                        for delz in [-0.5, 0, 0.5]:
                            new_node = (pos[0] + delx, pos[1] + dely, pos[2] + delz)
                            if self.graph.in_bounds(new_node):
                                self.graph.walls.update([new_node])
                                self.update_node(new_node)

                                # self.update_nodes([node for node in self.graph.neighbors((pos[0] + delx, pos[1] + dely))
                                #                        if node not in self.graph.walls])

                                if robot_plot:
                                    ax.plot([self.real_world_multiplier * new_node[0]],
                                            [self.real_world_multiplier * new_node[1]],
                                            [self.real_world_multiplier * new_node[2]], 'o', color='gray')
                                self.seen.update([new_node])

                pos = (
                    np.round(wall[0] * dnom) / dnom, np.round(wall[1] * dnom) / dnom, np.floor(wall[2] * 2) / (2))

                # for delx in [0, frac]:
                #     for dely in [0, frac]:
                #         for delz in [-0.5, 0, 0.5]:
                # new_node = (pos[0] + delx, pos[1] + dely, pos[2] + delz)
                new_node = pos
                if self.graph.in_bounds(new_node):
                    self.graph.walls.update([new_node])
                    self.update_node(new_node)

                    # self.update_nodes([node for node in self.graph.neighbors((pos[0] + delx, pos[1] + dely))
                    #                        if node not in self.graph.walls])

                    if robot_plot:
                        ax.plot([self.real_world_multiplier * new_node[0]],
                                [self.real_world_multiplier * new_node[1]],
                                [self.real_world_multiplier * new_node[2]], 'o', color='gray')
                    self.seen.update([new_node])

    def move_to_goal(self):
        ax, ax_label = self.initialise_plot()
        observation = self.real_graph.observe(self.position, self.view_range)  # Observe the surroundings in the view
        # range
        walls = self.graph.new_walls(
            observation)  # Define the position of newly observed walls to the graph seen by the robot
        self.graph.update_walls(walls)  # Update the walls on the robot's map

        self.adapt_grid(walls, ax, robot_plot=False)

        for n in list(observation.keys()):
            if n in self.graph.available_nodes:
                self.seen.update([n])

        # Compute the shortest path to the goal based on the current information
        self.compute_shortest_path()
        last_node = self.position  # Define last node as the current position

        yield self.position, observation, self.graph.walls

        # Initialise the plot axes and draw the grid
        # label_grid3(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal)
        draw_grid3(self.graph, ax, self.view_range, path=self.position, goal=self.goal)

        while self.position != self.start or len(
                self.seen) != len(self.graph.available_nodes):  # while the goal has not been reached
            condition = True
            
            # If the robot becomes stuck in a position with inf g, there is no path that can be taken
            if self.rhs(self.position) == float('inf'):
                raise Exception("No path")

            best_step = self.position
            neighbors = self.graph.neighbors(self.position)

            for neighbor in neighbors:
                if self.g(neighbor) + distance3(neighbor, self.position) * 2 <= self.g(best_step) + distance3(best_step,
                                                                                                              self.position) * 2:
                    best_step = neighbor

            self.position = best_step
            observation = self.real_graph.observe(self.position, self.view_range)  # make a new observation
            new_walls = self.graph.new_walls(observation)  # obtain the new walls from the observation

            newly_seen = []
            for n in list(observation.keys()):
                if n in self.graph.available_nodes:
                    if n not in self.seen:
                        newly_seen.append(n)
                    self.seen.update([n])


            # If new walls have been observed, update walls, the key modifier and nodes
            if new_walls:
                self.graph.update_walls(new_walls)
                # self.update_node(self.position)
                self.Km += heuristic3(last_node,
                                      self.position)  # updates the key modifier by how much the robot has moved since the last time it saw a wall
                last_node = self.position
                # Update all nodes that are either in new_walls or their neighbors (if they aren't walls themselves)
                self.adapt_grid(new_walls, ax, robot_plot=False)

                self.update_node(self.position)
                self.update_node(self.start)
                self.compute_shortest_path()  # Recompute the shortest path

            elif len(self.seen) == len(self.graph.available_nodes):  # If all points have been seen
                if self.position == self.start:
                    print(len(self.graph.available_nodes))
                    print(self.graph.height)
                    print(self.graph.width)
                    pass
                else:
                    self.update_node(self.position)
                    self.update_node(self.start)
                    self.compute_shortest_path()  # Recompute the shortest path

            else:

                self.update_node(self.position)
                self.G_VALS[self.position] = self.rhs(self.position)
                
                for n in self.graph.neighbors(self.position):
                    self.update_node(n)
                    self.G_VALS[n] = self.rhs(n)

                self.update_node(self.position)
                self.G_VALS[self.position] = self.rhs(self.position)

                # Condition only True if the robot does not have unvisited nodes just outside its field of vision
                if condition:
                    # self.update_node(self.position)
                    self.update_node(self.start)
                    self.compute_shortest_path()  # Recompute the shortest path

            yield self.position, observation, self.graph.walls
            # label_grid3(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal)
            draw_grid3(self.graph, ax, self.view_range, path=self.position, goal=self.goal)

if __name__ == "__main__":
    # GRAPH, START, END = grid_from_string3("""
    #     .....
    #     .###.
    #     ..o#.
    #     .....
    #     ...##
    #     ....X L
    #     #####
    #     #####
    #     #####
    #     #####
    #     ####. L
    #     .....
    #     .###.
    #     ...#.
    #     .....
    #     ...##
    #     .....
    #     """, 5, 5, 3)
    #
    # y1 = np.arange(1, 2.6, 0.1)
    #
    # for i in range(len(y1)):
    #     GRAPH.walls.add((1.1, y1[i], 0))

    GRAPH, START, END = grid_from_string3("""
                   ########
                   #......#
                   #......#
                   #......#
                   #......#
                   #......#
                   #......#
                   #o.....#
                   #......#
                   #......#
                   #.....X#
                   ########
                    """, 8, 12, 2)

    y1 = np.arange(1, 7.6, 0.1)

    for i in range(len(y1)):
        GRAPH.walls.add((4.4, y1[i], 0))

    y2 = np.arange(7.5, 11.1, 0.1)

    for i in range(len(y2)):
        GRAPH.walls.add((3.2, y2[i], 0))

    y3 = np.arange(9, 11.1, 0.1)

    for i in range(len(y3)):
        GRAPH.walls.add((1.7, y3[i], 0))

    y4 = np.arange(0, 11.1, 0.1)

    for i in range(len(y4)):
        GRAPH.walls.add((0, y4[i], 0))
        GRAPH.walls.add((7, y4[i], 0))

    x1 = np.arange(3, 5.3, 0.1)

    for i in range(len(x1)):
        GRAPH.walls.add((x1[i], 7.5, 0))

    x2 = np.arange(6, 7.1, 0.1)

    for i in range(len(x2)):
        GRAPH.walls.add((x2[i], 7.5, 0))

    x3 = np.arange(0, 7.1, 0.1)

    for i in range(len(x3)):
        GRAPH.walls.add((x3[i], 11, 0))
        GRAPH.walls.add((x3[i], 0, 0))

    # GRAPH, START, END = grid_from_string3("""
    #     .....
    #     .###.
    #     ..o#.
    #     .....
    #     ...##
    #     ....X L
    #     .....
    #     .###.
    #     ...#.
    #     .....
    #     ...##
    #     .....
    #     """, 5,5,2)

    # GRAPH, START, END = grid_from_string3("""
    #     .....
    #     .#...
    #     ..o..
    #     .....
    #     ...##
    #     ....X
    #     """, 5,5,1)

    # GRAPH, START, END = grid_from_string3("""
    #         ...
    #         .o.
    #         ..X
    #         """, 3, 3, 1)


    dstar = greedyDStarLite3(GRAPH, START,
                             END)  # Define an instance of the greedyDStarLite3 class using the graph object and the start and end positions

    
    GRAPH, START, END = grid_from_string3("""
            oX.................
            ....##############.
            ...................
            ................... L
            ...................
            ....##############.
            ...................
            ................... L
            ...................
            ....##############.
            ...................
            ................... L
            """, 19, 4, 3)
    


    dstar = greedyDStarLite3(GRAPH, START, END)  # Define an instance of the greedyDStarLite3 class using the graph object and the start and end positions


    seen = [p for p, o, w in dstar.move_to_goal()]

    plt.show()

    # for y in range(GRAPH.height):
    #     for x in range(GRAPH.width):
    #         if (x, y) not in seen:
    #             print('False')
    #             break

    print(seen)
