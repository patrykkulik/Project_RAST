# utility functions for dealing with square grid
import numpy as np
import matplotlib.pyplot as plt

import adaptive_grid
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


def draw_grid(graph, ax, view_range, real_world_multiplier,**style):
    # for wall in graph.walls:
    #     # if wall in graph.available_nodes:
    #     ax.plot(3*wall[0], 3*wall[1], 'ko')
        # px = wall[0]
        # py = wall[1]
        # if (px + 1, py) in graph.walls:
        #     ax.plot([px, px+1], [py, py], 'k')
        # if (px - 1, py) in graph.walls:
        #     ax.plot([px, px-1], [py, py], 'k')
        # if (px, py+1) in graph.walls:
        #     ax.plot([px, px], [py, py+1], 'k')
        # if (px, py-1) in graph.walls:
        #     ax.plot([px, px], [py, py-1], 'k')


    ax.plot(real_world_multiplier*style['goal'][0], real_world_multiplier*style['goal'][1], 'o', color='yellow')

    px = real_world_multiplier*style['path'][0]
    py = real_world_multiplier*style['path'][1]
    point = ax.plot(px, py, 'ro')
    xs = np.arange(- real_world_multiplier*view_range, real_world_multiplier*view_range, 0.001)
    ys = np.sqrt((real_world_multiplier*view_range)**2 - xs**2)
    ys2 = -ys
    visibility = ax.plot(np.append(xs,xs[::-1]) + px, np.append(ys,ys2[::-1]) + py, color = 'gray')
    plt.pause(0.5)
    point.pop(0).remove()
    visibility.pop(0).remove()


    # for y in range(graph.height):
    #     for x in range(graph.width):
    #         print("%%-%ds" % width % draw_tile(graph, (x, y), style, width),
    #               end="")
    #     print()


def label_grid(graph, ax, g_vals, rhs_vals, position, goal, real_world_multiplier):
    ax.clear()

    ax.set_ylim(0, real_world_multiplier*graph.height)
    ax.set_xlim(0, real_world_multiplier*graph.width)
    ax.xaxis.set_major_locator(plt.MultipleLocator(real_world_multiplier))
    ax.yaxis.set_major_locator(plt.MultipleLocator(real_world_multiplier))
    ax.grid()

    # (px, py) = 3*position[0], 3*position[1]
    # ax.fill([px, px, px + 1, px + 1], [py, py + 1, py + 1, py], color='green')


    for y in range(graph.height):
        for x in range(graph.width):
            if (x,y) in g_vals.keys():
                label_g = ax.text(real_world_multiplier*x+0.4,real_world_multiplier*y+0.75, str(np.round(g_vals[(x,y)],2)))
            else:
                label_g = ax.text(real_world_multiplier*x + 0.4, real_world_multiplier*y + 0.75, '\u221e')
            if (x,y) in rhs_vals.keys():
                label_rhs = ax.text(real_world_multiplier*x + 0.4, real_world_multiplier*y + 0.25, str(np.round(rhs_vals[(x, y)],2)))
            else:
                label_rhs = ax.text(real_world_multiplier*x + 0.4, real_world_multiplier*y + 0.25, '1')


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
    g = adaptive_grid.SquareGrid(len(lines[0]), len(lines))  # Create a Grid that is equivalent to the input grid
    start, end = None, None

    for row, line in enumerate(lines):      # Get the current row and the current line of the grid
        for col, char in enumerate(line):   # Get the current column and the current character of the grid

            if char == adaptive_grid.WALL:           # If the character is the same as a defined wall character,
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


