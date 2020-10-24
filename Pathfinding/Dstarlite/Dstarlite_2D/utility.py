# utility functions for dealing with square grid
import numpy as np
import matplotlib.pyplot as plt

import grid
from priority_queue import PriorityQueue


def from_id_width(id, width):
    return id % width, id // width


def draw_tile(graph, id, style, width):
    r = "."
    if ('number' in style
          and id in style['number']
          and style['number'][id] != float('inf')):
        r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = "\u2192"
        if x2 == x1 - 1: r = "\u2190"
        if y2 == y1 + 1: r = "\u2193"
        if y2 == y1 - 1: r = "\u2191"
    if 'start' in style and id == style['start']:
        r = "o"
    if 'goal' in style and id == style['goal']:
        r = "X"
    if 'path' in style and id == style['path']:  # 'in' if you want to plot all at onece. Otherwise: '=='
        r = "∆"
    if id in graph.walls: r = "#" * max(width-1, 1)
    return r


def draw_grid(graph, ax, width=2, **style):

    for y in range(graph.height):
        for x in range(graph.width):
            s = draw_tile(graph, (x, y), style, width)
            if s == '∆':
                point = ax.plot(x+0.5,y+0.5,'ro')
                plt.pause(1)
                point.pop(0).remove()
            elif s == '##':
                ax.fill([x, x, x+1, x+1], [y, y+1, y+1, y], 'k')
            elif s == 'X':
                ax.fill([x, x, x+1, x+1], [y, y+1, y+1, y], color='yellow')


    # for y in range(graph.height):
    #     for x in range(graph.width):
    #         print("%%-%ds" % width % draw_tile(graph, (x, y), style, width),
    #               end="")
    #     print()


def label_grid(graph, ax, g_vals, rhs_vals, back_pointers, position, goal):
    ax.clear()

    ax.set_ylim(0, graph.height)
    ax.set_xlim(0, graph.width)
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))
    ax.yaxis.set_major_locator(plt.MultipleLocator(1))
    ax.grid()

    (px, py) = position
    ax.fill([px, px, px + 1, px + 1], [py, py + 1, py + 1, py], color='green')

    while position != goal:
        (px, py) = back_pointers[position]
        ax.fill([px, px, px + 1, px + 1], [py, py + 1, py + 1, py], color='green')
        position = (px,py)

    for y in range(graph.height):
        for x in range(graph.width):
            if (x,y) in g_vals.keys():
                label_g = ax.text(x+0.4,y+0.75, str(g_vals[(x,y)]))
            else:
                label_g = ax.text(x + 0.4, y + 0.75, '\u221e')
            if (x,y) in rhs_vals.keys():
                label_rhs = ax.text(x + 0.4, y + 0.25, str(rhs_vals[(x, y)]))
            else:
                label_rhs = ax.text(x + 0.4, y + 0.25, '\u221e')


def reconstruct_path(came_from, start, goal):
    """Reconstruct a shortest path from a dictionary of back-pointers"""
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.append(start)  # optional
    path.reverse()  # optional
    return path


def grid_from_string(string):
    """
    Construct a SquareGrid from a string representation
    Representation:
    . - a passable square
    o - the start position
    X - the goal position
    # - an unpassable square (a wall)
    Args:
        :type string: str
    Returns a 3-tuple: (g: SquareGrid, start: Tuple, end: Tuple)
    """
    assert string.count('o') == 1, "Cant have more than 1 start position!"
    assert string.count('X') == 1, "Cant have more than 1 end position!"
    lines = [l.strip() for l in string.split('\n') if l.strip()]  # Strips all the whitespace from the strings
    g = grid.SquareGrid(len(lines[0]), len(lines))  # Create a Grid that is equivalent to the input grid
    start, end = None, None

    for row, line in enumerate(lines):      # Get the current row and the current line of the grid
        for col, char in enumerate(line):   # Get the current column and the current character of the grid

            if char == grid.WALL:           # If the character is the same as a defined wall character,
                g.walls.add((col, row))     # add its position to the wall set

            if char == 'o':                 # If the character corresponds to the start character,
                start = (col, row)          # define the start position

            if char == 'X':                 # If the character corresponds to the end character,
                end = (col, row)            # define the end position

    # Make sure that start and end exist
    assert start is not None
    assert end is not None
    return g, start, end    # return the grid object, start and end


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def a_star_search(graph, start, goal):
    """A star algorithm courtesy of http://www.redblobgames.com/pathfinding/
    Used here to confirm a path is traversable"""
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.pop()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

