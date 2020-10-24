from collections import deque
import matplotlib.pyplot as plt
import numpy as np

# Import needed functions from other scripts:
from utility_field import draw_grid
from utility_field import label_grid
from utility_field import grid_from_string
from priority_queue import PriorityQueue
from grid_field import AgentViewGrid, SquareGrid


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
    def __init__(self, graph, start, goal, view_range=1.5):
        # Initialise the Field D* algorithm:

        self.graph = AgentViewGrid(graph.width, graph.height)  # Define the graph method as an instance of AgentView
        # (the map from the perspective of the robot)
        self.real_graph: SquareGrid = graph  # Make the real_graph method equal to the real version of the grid
        self.view_range = view_range  # Define the range of view of the robot

        self.seen = set()  # Create a set of all the seen nodes

        # Create dictionaries:
        self.back_pointers = {}  # Dictionary describing the first guess for the path of least resistance
        self.G_VALS = {start: 1000,
                       goal: float('inf')}  # Dictionary of all g values for all of the expanded cells

        self.Km = 0  # Key modifier. When the robot makes a move, this is incremented so that heuristic? can stay same
        self.position = start  # Starting position of the robot
        self.start = start
        self.goal = (500, 500)  # Goal position (fake)
        self.RHS_VALS = {start: 1000, self.goal: 0}  # Dictionary of all rhs values for all of the expanded cells

        self.frontier = PriorityQueue()  # Define the frontier method as an instance of PriorityQueue
        self.frontier.put(self.goal, self.calculate_key(self.goal))  # Append the goal and its key to the frontier

    def initialise_plot(self):
        plt.figure()
        plt.grid()
        ax = plt.subplot(1, 2, 1)  # axis showing the movement of the robot
        ax.grid()
        ax_label = plt.subplot(1, 2, 2)  # axis showing the labels of the nodes

        # Set the appropriate axes limits
        ax.set_ylim(-0.5, self.graph.height - 0.5)
        ax.set_xlim(-0.5, self.graph.width - 0.5)

        # set a tick increment to 1 for both axes:
        ax.xaxis.set_major_locator(plt.MultipleLocator(1))
        ax.yaxis.set_major_locator(plt.MultipleLocator(1))
        plt.pause(3)

        return ax, ax_label

    def rhs(self, node):
        """
        Return node's rhs value. If node not in the RHS_VALS dict, return inf. If node is the goal node, return 0
        """
        # rhs value is what the g value of the node should be
        if node == self.goal:
            return 0     # The goal node should always be zero
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
        if node[0] % 1 == 0 and node[1] % 1 == 0:  # If both coordinates are integers, check the vertex
            if node in self.graph.walls:
                return True
            else:
                return False
        # Otherwise, check the verticies at the ends of the current edge
        elif node[0] % 1 != 0:
            if (np.floor(node[0]), node[1]) in self.graph.walls and (np.ceil(node[0]), node[1]) in self.graph.walls:
                return True
            else:
                return False
        elif node[1] % 1 != 0:
            if (node[0], np.floor(node[1])) in self.graph.walls and (node[0], np.ceil(node[1])) in self.graph.walls:
                return True
            else:
                return False
        else:
            return False

    def feasible_motion(self, from_node, to_node):
        """ Function to check if motion between two nodes is feasible (no obstacle present in between) """
        # Could be improved to make it less convoluted
        if from_node[0] % 1 != 0 and to_node[0] % 1 != 0 and to_node[1] != from_node[1]:
            px = np.round((from_node[0] + to_node[0])/2)
            py = (from_node[1] + to_node[1])/2
            return not self.obstacle_check((px, py))
        elif from_node[1] % 1 != 0 and to_node[1] % 1 != 0 and to_node[0] != from_node[0]:
            py = np.round((from_node[1] + to_node[1])/2)
            px = (from_node[0] + to_node[0])/2
            return not self.obstacle_check((px, py))
        elif from_node[0] % 1 != 0:
            if abs(to_node[0] - from_node[0]) > 1:
                py = (from_node[1] + to_node[1]) / 2
                px = np.round(from_node[0])
                return not self.obstacle_check((px, py))
            elif abs(to_node[1] - from_node[1]) > 1:
                px = (from_node[0] + to_node[0]) / 2
                py = np.round(from_node[1])
                return not self.obstacle_check((px, py))
            else:
                return True
        elif from_node[1] % 1 != 0:
            if abs(to_node[1] - from_node[1]) > 1:
                px = (from_node[0] + to_node[0]) / 2
                py = np.round(from_node[1])
                return not self.obstacle_check((px, py))
            elif abs(to_node[0] - from_node[0]) > 1:
                px = (from_node[1] + to_node[1]) / 2
                py = np.round(from_node[0])
                return not self.obstacle_check((px, py))
            else:
                return True
        else:
            return True

    def compute_cost(self, s,  sa, sb, diagonals, multiplier):
        """Compute cost function developed by Dave Ferguson and Anthony Stentz
        (https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_4/ferguson_david_2005_4.pdf)
        :return vs: rhs of node s from edge sa <-> sb
         destination: the destination to obtain optimal cost
        """
        # Function is used here to compute the rhs for the current node (s) if motion is allowed from anywhere on the
        # sa <-> sb edge. This cost is computed using linear interpolation

        destination = None

        if sa in diagonals:  # if sa is a diagonal neighbor of s
            s1 = sb
            s2 = sa
        else:
            s1 = sa
            s2 = sb

        if self.obstacle_check(s1) and self.obstacle_check(s2):
            c = float('inf')*multiplier
            b = float('inf')*multiplier
        else:
            c = 2
            b = 2

        if min(c, b) == float('inf'):
            vs = float('inf')
            destination = None
        elif self.g(s1) <= self.g(s2):
            vs = min(c, b) + self.g(s1)
            destination = s1
        else:
            f = self.g(s1) - self.g(s2)
            if f <= b:
                if c <= f:
                    vs = c * distance(s, s2) + self.g(s2)
                    destination = s2
                else:
                    # y = min(f / np.sqrt(c ** 2 - f ** 2), 1)
                    #
                    # if s1[0] == s2[0] and s2[1] > s1[1]:
                    #     destination = tuple(np.round((s1[0], s1[1]+y), 2))
                    # elif s1[0] == s2[0] and s2[1] < s1[1]:
                    #     destination = tuple(np.round((s1[0], s1[1] - y), 2))
                    # elif s1[1] == s2[1] and s2[0] > s1[0]:
                    #     destination = tuple(np.round((s1[0] + y, s1[1]), 2))
                    # elif s1[1] == s2[1] and s2[0] < s1[0]:
                    #     destination = tuple(np.round((s1[0] - y, s1[1]), 2))
                    #
                    # vs = c * distance(destination, s) + f * (1 - y) + self.g(s2)
                    vs = c * distance(s, s1) + self.g(s1)
                    destination = s1
            else:
                if c <= b:
                    vs = c * distance(s, s2) + self.g(s2)
                    destination = s2
                else:
                    x = 1 - min(b / np.sqrt(c ** 2 - b ** 2), 1)
                    vs = c * np.sqrt(1 + (1 - x) ** 2) + b * x + self.g(s2)
                    destination = s2

        return vs, destination

    def calculate_rhs(self, node):
        """ Calculate the rhs value of node """
        multiplier = 1

        if self.obstacle_check(node):  # If node is in an obstacle, rhs is equal to inf
            return float('inf')
        else:
            # Round the node to the nearest integer node - this might reduce the optimality of the path but will largely
            # reduce the computational cost as the node's neighbors will always be taken as full integer nodes
            px = np.round(node[0])
            py = np.round(node[1])
            if self.obstacle_check((px, py)):
                # If the rounding puts the node in an obstacle node, try rounding the opposite way
                if node[0] % 1 == 0:
                    px = node[0]
                    if np.ceil(node[1]) == np.round(node[1]):
                        py = np.floor(node[1])
                    else:
                        py = np.ceil(node[1])
                elif node[1] % 1 == 0:
                    py = node[1]
                    if np.ceil(node[0]) == np.round(node[0]):
                        px = np.floor(node[0])
                    else:
                        px = np.ceil(node[0])

            if self.obstacle_check((px, py)):  # If the node is still in the obstacle, return inf
                return float('inf')
            elif (px, py) in self.seen:
                multiplier = 2

            # Define diagonal neighbors of the node
            diagonals = [(px + 1, py + 1), (px - 1, py + 1), (px - 1, py - 1),
                         (px + 1, py - 1)]

            # Compute all local neighbors of the node
            local_neighbors = self.graph.neighbors(node)
            connbrs = []  # The connbrs list will define pairs of adjescent node (essentially defining 8 edges around
            # the given node)
            for i in range(len(local_neighbors) - 1):
                if heuristic(local_neighbors[i], local_neighbors[i + 1]) == 1:
                    connbrs.append((local_neighbors[i], local_neighbors[i + 1]))

            # In case, that some of the neighboring edges are outside the scope of the graph, check if the last pair of
            # neighbor nodes is a distance of 1 away from each other
            if heuristic(local_neighbors[len(local_neighbors) - 1], local_neighbors[0]) == 1:
                connbrs.append((local_neighbors[len(local_neighbors) - 1], local_neighbors[0]))

            # Set the value of vs as inf. This will be overridden with the rhs value for the node
            vs = float('inf')
            for pair in connbrs:  # For every pair in the connbrs list

                vs_temp, destination = self.compute_cost(node, pair[0], pair[1], diagonals, multiplier)  # compute the cost

                # If the cost is smaller than the current value, check if the motion is feasible
                if vs_temp < vs and self.feasible_motion(node, destination):
                    if destination in self.back_pointers.keys():  # if the destination is in back_pointers dictionary
                        if self.back_pointers[destination] != node:  # if the pointer points to current node, we will get a circular definition. (I think)
                            self.back_pointers[node] = destination
                            vs = vs_temp
                    else:
                        self.back_pointers[node] = destination
                        vs = vs_temp

            if vs != float('inf'):  # If we found a value that is less than infinity, update the node
                if self.back_pointers[node][0] % 1 != 0 or self.back_pointers[node][1] % 1 != 0:
                    if destination in self.back_pointers.keys():
                        if self.back_pointers[destination] != node:
                            self.update_node(self.back_pointers[node])
                    else:
                        self.update_node(self.back_pointers[node])

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

            if len(last_nodes) == 10 and len(set(last_nodes)) < 3: # if the last_nodes list is full but contains less
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

                    keys = list(self.RHS_VALS.keys())
                    for key in keys:
                        # update all the non integer positions that were previously expanded near the node
                        if \
                            node[0] - 1 <= key[0] <= node[0] + 1 and node[1] - 1 <= key[1] <= \
                                        node[1] + 1 and (key[0] % 1 != 0 or key[1] %1 != 0):
                            self.update_node(key)
                else:
                    self.update_nodes(self.graph.neighbors((4,4)))

            # If the g(node) < rhs(node) we have a difficult situation where we first have to set the g(node) = inf,
            # after which we move the node and its neighbors back to the frontier. This ensures that the node is not
            # updated more than necessary. After the required nodes move through the queue and this node is being
            # expanded, the situation becomes a simple case where the g(node) is relaxed to its rhs value
            else:
                self.G_VALS[node] = float('inf')
                if node != self.goal:
                    for n in self.graph.neighbors(node):
                        self.update_node(n)

                    keys = list(self.RHS_VALS.keys())
                    for key in keys:
                        # update all the non integer positions that were previously expanded near the node
                        if \
                                node[0] - 1 <= key[0] <= node[0] + 1 and node[1] - 1 <= key[1] <= \
                                        node[1] + 1 and (key[0] % 1 != 0 or key[1] %1 != 0):
                            self.update_node(key)

                    self.update_node(node)
                else:
                    self.update_node(node)
                    self.update_nodes(self.graph.neighbors((4,4)))


            if self.frontier.empty():
                break

        return self.back_pointers.copy(), self.G_VALS.copy()

    def move_to_goal(self):
        observation = self.real_graph.observe(self.position, self.view_range)  # Observe the surroundings in the view
        # range
        walls = self.graph.new_walls(observation)  # Define the position of newly observed walls to the graph seen by
        # the robot
        self.graph.update_walls(walls)  # Update the walls on the robot's map

        newly_seen = []
        for n in self.graph.neighbors(self.position):
            self.seen.update([n])
            newly_seen.append(n)

        # Compute the shortest path to the goal based on the current information
        self.compute_shortest_path()
        last_node = self.position  # Define last node as the current position

        yield self.position, observation, self.graph.walls, newly_seen

        # Initialise the plot axes and draw the grid
        ax, ax_label = self.initialise_plot()
        label_grid(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal)
        draw_grid(self.graph, ax, self.view_range, path=self.position, goal=self.goal)


        while self.position != self.start or len(self.seen) != self.graph.height * self.graph.width:  # while the goal has not been reached
            condition = True
            # If the robot becomes stuck in a position with inf g, there is no path that can be taken
            # if self.rhs(self.position) == float('inf'):
            #     raise Exception("No path")

            best_step = self.position
            neighbors = []
            if self.position in self.back_pointers.keys():
                if self.feasible_motion(self.position, self.back_pointers[self.position]):
                    best_step = self.back_pointers[self.position]

            for key in self.G_VALS.keys():
                if key == self.position or self.obstacle_check(key) or key[0] % 1 != 0 or key[1] % 1 != 0:
                    continue
                elif self.position[0]-1 <= key[0] <= self.position[0]+1 and self.position[1] - 1 <= key[1] <= \
                        self.position[1] + 1:
                    if self.position[0] % 1 != 0 or self.position[1] % 1 != 0:
                        if not self.feasible_motion(self.position, key):
                            continue
                    neighbors.append(key)

            for neighbor in neighbors:
                if self.g(neighbor) + distance(neighbor, self.position)*2 <= self.g(best_step) + distance(best_step, self.position)*2:
                    best_step = neighbor

            self.position = best_step
            observation = self.real_graph.observe(self.position, self.view_range)  # make a new observation
            new_walls = self.graph.new_walls(observation)  # obtain the new walls from the observation
            newly_seen = []
            for n in self.graph.neighbors(self.position):
                self.seen.update([n])
                newly_seen.append(n)


            # If new walls have been observed, update walls, the key modifier and nodes
            if new_walls:
                self.graph.update_walls(new_walls)
                # self.update_node(self.position)
                self.Km += heuristic(last_node, self.position)  # updates the key modifier by how much the robot has
                # moved since the last time it saw a wall
                last_node = self.position
                # Update all nodes that are either in new_walls or their neighbors (if they aren't walls themselves)
                for wallnode in new_walls:
                    self.update_nodes([node for node in self.graph.neighbors(wallnode)
                                       if node not in self.graph.walls])

                self.update_node(self.position)
                self.update_node(self.start)
                self.compute_shortest_path()  # Recompute the shortest path

            elif len(self.seen) == self.graph.height * self.graph.width:  # If all points have been seen
                self.update_node(self.position)
                self.update_node(self.start)
                self.compute_shortest_path()  # Recompute the shortest path

            else:

                for n in self.graph.neighbors(self.position):
                    self.update_node(n)
                    self.G_VALS[n] = self.rhs(n)

                if len(newly_seen) == 0:
                    keys = list(self.RHS_VALS.keys())
                    for key in keys:
                        # update all the non integer positions that were previously expanded near the node
                        if \
                                self.position[0] - (self.view_range+1) <= key[0] <= self.position[0] + self.view_range+1 and self.position[1] - (self.view_range+1) <= key[1] <= \
                                        self.position[1] + self.view_range+1:
                            if self.RHS_VALS[key] == 1:
                                condition = False
                                break

                # Condition only True if the robot does not have unvisited nodes just outside its field of vision
                if condition:
                    self.update_node(self.position)
                    self.update_node(self.start)
                    self.compute_shortest_path()  # Recompute the shortest path


            yield self.position, observation, self.graph.walls, newly_seen
            label_grid(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.position, self.goal)
            draw_grid(self.graph, ax, self.view_range, path=self.position, goal=self.goal)


if __name__ == "__main__":
    # GRAPH, START, END = grid_from_string("""
    #     .....
    #     .###.
    #     ..o#.
    #     .....
    #     ...##
    #     ....X
    #     """)

    # GRAPH, START, END = grid_from_string("""
    #     .....
    #     .###.
    #     ..o#.
    #     ...#.
    #     .####
    #     ....X
    #     """)

    # GRAPH, START, END = grid_from_string("""
    #      .......
    #      ..####.
    #      ....o#.
    #      ..####.
    #      ..#..#.
    #      .....#.
    #      .....#X
    #      .....##
    #      """)

    # GRAPH, START, END = grid_from_string("""
    #     ..######
    #     ......o#
    #     ..######
    #     .......X
    #     """)

    # GRAPH, START, END = grid_from_string("""
    #     .o#.
    #     ....
    #     ....
    #     ..##
    #     ...X
    #     """)

    # GRAPH, START, END = grid_from_string("""
    #          ..........
    #          ...######.
    #          .......o#.
    #          ...######.
    #          ...#....#.
    #          ...#....#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#.
    #          ........#X
    #          ........##
    #          """)

    GRAPH, START, END = grid_from_string("""
            .############
            .#....#.....#
            .##.###.#...#
            .#......#...#
            .#......#...#
            .#......#.###
            .#......#...#
            .#......#...#
            .#......#...#
            ..o.........#
            .#...########
            .#......#...#
            .#...#..#.X.#
            .#...#..#...#
            .############
            .............
             """)

    # GRAPH, START, END = grid_from_string("""
    #             ##############
    #             #X.#######..##
    #             ###.########.#
    #             ##.########.##
    #             #.#.######.#.#
    #             #.#.######.#.#
    #             #.#.#.##.#.#.#
    #             ##.##.##.##.##
    #             ............o#
    #             ##############
    #              """)


    dstar = FieldDStarLite(GRAPH, START, END)  # Define an instance of the DStarLite class using the graph object and
    # the start and end positions
    seen = []
    for p, o, w, s in dstar.move_to_goal():
        for i in s:
            seen.append(i)
    # seen = [s for p, o, w, s in dstar.move_to_goal()]
    plt.show()

    for y in range(GRAPH.height):
        for x in range(GRAPH.width):
            if (x, y) not in seen:
                print('False')
                break

    print(seen)
