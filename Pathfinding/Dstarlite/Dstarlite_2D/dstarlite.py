from collections import deque
from functools import partial

import matplotlib.pyplot as plt


from utility import draw_grid
from utility import label_grid
from priority_queue import PriorityQueue
from grid import AgentViewGrid, SquareGrid
from utility import grid_from_string


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)  # Raw distance between two grid points


class DStarLite(object):
    def __init__(self, graph, start, goal, view_range=1):
        # Init the graphs
        self.graph = AgentViewGrid(graph.width, graph.height)  # Define the graph method as an instance of AgentView
        self.real_graph: SquareGrid = graph  # Make the real_graph method equal to the real version of the grid
        self.view_range = view_range  # Define the range of view of the robot

        # Create dictionaries:
        self.back_pointers = {}  # Dictionary of all
        self.G_VALS = {}         # Dictionary of all g values for all of the expanded cells
        self.RHS_VALS = {}       # Dictionary of all rhs values for all of the expanded cells

        self.Km = 0  # Key modifier. When the robot makes a move, this is incremented so that heuristic? can stay same
        self.position = start   # Starting position of the robot
        self.goal = goal        # Goal position

        self.frontier = PriorityQueue()  # Define the frontier method as an instance of PriorityQueue
        self.frontier.put(self.goal, self.calculate_key(self.goal))  # Append the goal and its key to the frontier
        self.back_pointers[self.goal] = None  # Initialise back_pointers dictionary with the goal corresponding to None

    def rhs(self, node):
        # return the item corresponding to node in the RHS_VALS dict if the node is not equal to the goal node. If node
        # not present in the dict, return inf
        return self.RHS_VALS.get(node, float('inf')) if node != self.goal else 0

    def g(self, node):  # G is equivalent to a distance from start to current node
        # return the item corresponding to node in the RHS_VALS dict. If node not present in the dict, return inf
        return self.G_VALS.get(node, float('inf'))

    def calculate_key(self, node):  # key is a function that determines the optimal path (lower key == better path)
        g_rhs = min([self.g(node), self.rhs(node)])  # G and RHS are supposed to be equal so get the min of them

        return (
            g_rhs + heuristic(node, self.position) + self.Km,
            g_rhs
        )   # The key is a combination of the total function and the start distance

    def lookahead_cost(self, node, neighbour):
        return self.g(neighbour) + self.graph.cost(neighbour, node)

    def lowest_cost_neighbour(self, node):
        # I believe that this is just a fancy way of plugging all of the neighbors into the function and finding min.
        # Partial evaluates the function with only one of the inputs and the second input is provided in the return
        # line, where python evaluates the function properly
        cost = partial(self.lookahead_cost, node)
        return min(self.graph.neighbors(node), key=cost)

    def calculate_rhs(self, node):
        lowest_cost_neighbour = self.lowest_cost_neighbour(node)  # figure out the neighbor with the lowest cost
        self.back_pointers[node] = lowest_cost_neighbour       # make the node point in the direction of the lowest_cost
        return self.lookahead_cost(node, lowest_cost_neighbour)   # the rhs is equal to cost of the neighbor + cost of
        # moving to it

    def update_node(self, node):
        if node != self.goal:
            self.RHS_VALS[node] = self.calculate_rhs(node)
        self.frontier.delete(node)  # remove the node from the frontier
        if self.g(node) != self.rhs(node):  # if the node is not consistent, place it back in the frontier
            self.frontier.put(node, self.calculate_key(node))

    def update_nodes(self, nodes):
        [self.update_node(n) for n in nodes]

    def compute_shortest_path(self):
        last_nodes = deque(maxlen=10)  # List like object that can be appended and popped from both sides.

        # As long as the key in the frontier are better than at my current position or I'm in the position where g!=rhs
        while self.frontier.first_key() < self.calculate_key(self.position) or self.rhs(self.position) != self.g(
                self.position):

            k_old = self.frontier.first_key()   # The old key is the smallest key from the frontier
            node = self.frontier.pop()          # The node is equal to the node in frontier with smallest key
            last_nodes.append(node)             # Append this node to last_nodes
            if len(last_nodes) == 10 and len(set(last_nodes)) < 3:
                raise Exception("Fail! Stuck in a loop")

            k_new = self.calculate_key(node)    # Calculate the new key function for the current node

            if k_old < k_new:   # If the old key was better than the new key
                self.frontier.put(node, k_new)  # Append the node with the new key to the frontier

            # If the g(node) > rhs(node) we have a simple situation where we can just set g(node) = rhs(node) and update
            # neighbors
            elif self.g(node) > self.rhs(node):
                self.G_VALS[node] = self.rhs(node)
                self.update_nodes(self.graph.neighbors(node))

            # If the g(node) < rhs(node) we have a difficult situation where we first have to set the g(node) = inf,
            # after which we move the node and its neighbors back to the frontier. This ensures that the node is not
            # updated more than necessary. After the required nodes move through the queue and this node is being
            # expanded, the situation becomes a simple case where the g(node) is relaxed to its rhs value
            else:
                self.G_VALS[node] = float('inf')
                self.update_nodes(self.graph.neighbors(node) + [node])

        return self.back_pointers.copy(), self.G_VALS.copy()

    def move_to_goal(self):
        observation = self.real_graph.observe(self.position, self.view_range)  # Observe the surroundings in the view
        # range
        walls = self.graph.new_walls(observation)  # Define the position of newly observed walls to the graph seen by
        # the robot
        self.graph.update_walls(walls)             # Update the walls

        # Compute the shortest path to the goal based on the current information
        self.compute_shortest_path()
        last_node = self.position   # Define last node as the current position

        yield self.position, observation, self.graph.walls

        plt.figure()
        plt.grid()
        ax = plt.subplot(1, 2, 1)
        ax.grid()
        ax_label = plt.subplot(1, 2, 2)


        ax.set_ylim(0, self.graph.height)
        ax.set_xlim(0, self.graph.width)

        # plt.xticks(range(0, self.graph.height + 1, 1))
        # plt.yticks(range(0, self.graph.width + 1, 1))
        ax.xaxis.set_major_locator(plt.MultipleLocator(1))
        ax.yaxis.set_major_locator(plt.MultipleLocator(1))

        label_grid(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.back_pointers, self.position, self.goal)

        draw_grid(self.graph, ax, width=3, path=self.position, goal=self.goal)


        while self.position != self.goal:  # while the goal has not been reached

            # If the robot becomes stuck in a position with inf g, there is no path that can be taken
            if self.g(self.position) == float('inf'):
                raise Exception("No path")

            self.position = self.lowest_cost_neighbour(self.position)  # move in the direction of lowest cost
            observation = self.real_graph.observe(self.position, self.view_range)  # make a new observation
            new_walls = self.graph.new_walls(observation)  # obtain the new walls from the observation

            # If new walls have been observed, update walls, the key modifier and nodes
            if new_walls:
                self.graph.update_walls(new_walls)
                self.Km += heuristic(last_node, self.position)  # updates the key modifier by how much the robot has
                # moved since the last time it saw a wall
                last_node = self.position
                # Update all nodes that are either in new_walls or their neighbors (if they aren't walls themselves)
                self.update_nodes({node for wallnode in new_walls
                                   for node in self.graph.neighbors(wallnode)
                                   if node not in self.graph.walls})
                self.compute_shortest_path()  # Recompute the shortest path

                label_grid(self.graph, ax_label, self.G_VALS, self.RHS_VALS, self.back_pointers, self.position,
                           self.goal)
            yield self.position, observation, self.graph.walls
            draw_grid(self.graph, ax, width=3, path=self.position, goal = self.goal)


if __name__ == "__main__":
    GRAPH, START, END = grid_from_string("""
        .....
        .###.
        ..o#.
        ...#.
        ...#.
        ...#X
        """)
    # GRAPH, START, END = grid_from_string("""
    # ..........
    # ...######.
    # .......o#.
    # ...######.
    # ...#....#.
    # ...#....#.
    # ........#.
    # ........#.
    # ........#.
    # ........#X
    # """)
    dstar = DStarLite(GRAPH, START, END)  # Define an instance of the DStarLite class using the graph object and the
    # start and end positions

    path = [p for p, o, w in dstar.move_to_goal()]

    # print("The graph (o=Start, X=Goal)")
    # draw_grid(GRAPH, width=3, start=START, goal=END)
    # print("\n\nPath taken (∆ symbols)")
    # draw_grid(GRAPH, width=3, path=path)
    plt.show()
