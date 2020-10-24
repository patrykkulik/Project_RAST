#
# utility_field3.py
#
# Created on 28/05/2020
#
#

# utility functions for dealing with square grid
import numpy as np
import matplotlib.pyplot as plt

import grid_field3

def from_id_width(id, width):
    return id % width, id // width


# draw_tile(graph, id, style, width) unused

def draw_grid3(graph, ax, view_range, **style):
    # for wall in graph.walls:
    #     ax.plot([wall[0]], [wall[1]], [wall[2]], 'ko')


    ax.plot([style['goal'][0]], [style['goal'][1]], [style['goal'][2]], 'o', color='yellow')

    px = style['path'][0]
    py = style['path'][1]
    pz = style['path'][2]
    point = ax.plot([px], [py], [pz], 'ro')

    # CHECK: viewing circle (plotting only)
    xs = np.arange(- view_range, view_range, 0.001)

    ys = np.sqrt(view_range ** 2 - xs ** 2)
    ys2 = -ys
    zs = np.sqrt(view_range ** 2 - xs ** 2)
    zs2 = -zs

    visibility = ax.plot(np.append(xs, xs[::-1]) + px, np.append(ys, ys2[::-1]) + py, pz, color='gray')
    visibility2 = ax.plot(np.append(xs, xs[::-1]) + px, np.append(zs, zs2[::-1]) + pz, py, zdir='y', color='gray')
    visibility3 = ax.plot(np.append(xs, xs[::-1]) + px, np.append(zs, zs2[::-1]) + pz, py, zdir='y', color='gray')

    label_position = ax.text(0, 0, 0, 'Position: ({},{},{})'.format(px, py, pz), horizontalalignment='right', )

    plt.pause(0.005)
    point.pop(0).remove()
    visibility.pop(0).remove()
    visibility2.pop(0).remove()
    visibility3.pop(0).remove()
    label_position.remove()

def label_grid3(graph, ax, g_vals, rhs_vals, position, goal):
    """ WARNING: label_grid3 will not present well in 3D"""

    ax.clear()

    ax.set_xlim(0, graph.length)
    ax.set_ylim(0, graph.width)
    ax.set_zlim(0, graph.height)
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))
    ax.yaxis.set_major_locator(plt.MultipleLocator(1))
    ax.zaxis.set_major_locator(plt.MultipleLocator(1))
    ax.grid()

    (px, py, pz) = position
    # ax.fill([px, px, px + 1, px + 1], [py, py + 1, py + 1, py], color='green')

    for z in range(graph.height):
        for y in range(graph.width):
            for x in range(graph.length):
                # if (x,y,z) in g_vals.keys():
                #     label_g = ax.text(x+0.4,y+0.75, z, str(np.round(g_vals[(x,y,z)],2)))
                # else:
                #     label_g = ax.text(x + 0.4, y + 0.75, z, '\u221e')

                if (x, y, z) in rhs_vals.keys():
                    label_rhs = ax.text(x + 0.4, y + 0.25, z, str(np.round(rhs_vals[(x, y, z)], 2)))
                else:
                    label_rhs = ax.text(x + 0.4, y + 0.25, z, '\u221e')
                    
# reconstruct_path(came_from, start, goal) unused

def grid_from_string3(string, length, width, height):
    """
    Construct a SquareGrid3 from a string representation
    Representation:
    . - a passable square
    o - the start position
    X - the goal position
    # - an unpassable square (a wall)
    L - delimiter between z-direction layers
    Args:
        :type string: str
        :type length: int
        :type width: int
        :type height: int
    Returns a 3-tuple: (g: SquareGrid3, start: Tuple, end: Tuple)
    """

    assert string.count('o') == 1, "Cant have more than 1 start position!"
    assert string.count('X') == 1, "Cant have more than 1 end position!"

    g = grid_field3.SquareGrid3(length, width, height)  # Create a Grid that is equivalent to the input grid
    start, end = None, None

    layers = [l.strip() for l in string.split('L') if l.strip()]  # Cuts string into layers
    for layer in range(len(layers)):
        lines = [l2.strip() for l2 in layers[layer].split('\n') if l2.strip()]

        for row, line in enumerate(lines):  # Get the current row and the current line of the grid
            for col, char in enumerate(line):  # Get the current column and the current character of the grid

                if char == grid_field3.WALL:  # If the character is the same as a defined wall character,
                    g.walls.add((col, row, layer))  # add its position to the wall set

                if char == 'o':  # If the character corresponds to the start character,
                    start = (col, row, layer)  # define the start position

                if char == 'X':  # If the character corresponds to the end character,
                    end = (col, row, layer)  # define the end position

    # Make sure that start and end exist
    assert start is not None
    assert end is not None
    return g, start, end  # return the grid object, start and end

# heuristic(a, b) unused

# a_star_search(graph, start, goal) unused
