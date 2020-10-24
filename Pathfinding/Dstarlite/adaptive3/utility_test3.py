# utility functions for dealing with square grid
import numpy as np
import matplotlib.pyplot as plt

import adaptive_grid3

def from_id_width(id, width):
    return id % width, id // width



def draw_grid3(graph, ax, view_range, real_world_multiplier, real_world_multiplierZ, orientation, **style):
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

    ax.plot([real_world_multiplier * style['goal'][0]], [real_world_multiplier * style['goal'][1]],
            [real_world_multiplierZ * style['goal'][2]], 'o', color='yellow')

    px = real_world_multiplier * style['path'][0]
    py = real_world_multiplier * style['path'][1]
    pz = real_world_multiplierZ * style['path'][2]
    point = ax.plot([px], [py], [pz], 'ro')

    xs = np.arange(- real_world_multiplier * view_range, real_world_multiplier * view_range, 0.001)
    ys = np.sqrt((real_world_multiplier * view_range) ** 2 - xs ** 2)
    ys2 = -ys
    zs = np.sqrt((real_world_multiplier * view_range) ** 2 - xs ** 2)
    zs2 = -zs

    # visibility2 = ax.plot(np.append(xs, xs[::-1]) + px, np.append(zs, zs2[::-1]) + pz, py, zdir='y', color='gray')
    # visibility3 = ax.plot(np.append(xs, xs[::-1]) + px, np.append(zs, zs2[::-1]) + pz, py, zdir='y', color='gray')

    xs = np.append(xs, xs[::-1])
    ys = np.append(ys, ys2[::-1])

    end = len(xs)
    if orientation == 0:
        xs = xs[int(end*0.5-0.073375*end):int(end*0.5+0.073375*end)]
        ys = ys[int(end*0.5-0.073375*end):int(end*0.5+0.073375*end)]
    elif orientation == 45:
        xs = xs[int(0.25*end):int(end*0.5)]
        ys = ys[int(0.25*end):int(end*0.5)]
    elif orientation == 90:
        xs = xs[int(0.073375*end):int(end*0.5-0.073375*end)]
        ys = ys[int(0.073375*end):int(end*0.5-0.073375*end)]
    elif orientation == 135:
        xs = xs[:int(end*0.25)]
        ys = ys[:int(end*0.25)]
    elif orientation == 180:
        xs = xs[:int(end*0.073375)]
        ys = ys[:int(end*0.073375)]
    elif orientation == -45:
        xs = xs[int(end*0.5):int(0.75)]
        ys = ys[int(end*0.5):int(0.75)]
    elif orientation == -90:
        xs = xs[int(end*0.5+0.073375*end):int(end*0.75+0.073375*end)]
        ys = ys[int(end*0.5+0.073375*end):int(end*0.75+0.073375*end)]
    elif orientation == -135:
        xs = xs[int(end*0.75):]
        ys = ys[int(end*0.75):]

    visibility = ax.plot(xs + px, ys + py, pz, color='gray')


    label_position = ax.text(0, 0, 0, 'Position: ({},{},{})'.format(px, py, pz), horizontalalignment='right', )

    plt.pause(0.01)
    point.pop(0).remove()
    visibility.pop(0).remove()
    # visibility2.pop(0).remove()
    # visibility3.pop(0).remove()
    label_position.remove()
    # for y in range(graph.height):
    #     for x in range(graph.width):
    #         print("%%-%ds" % width % draw_tile(graph, (x, y), style, width),
    #               end="")
    #     print()


def label_grid3(graph, ax, g_vals, rhs_vals, position, goal, real_world_multiplier, real_world_multiplierZ):
    ax.clear()

    ax.set_ylim(0, real_world_multiplier * graph.length)
    ax.set_xlim(0, real_world_multiplier * graph.width)
    ax.set_zlim(0, real_world_multiplierZ * graph.height)

    ax.xaxis.set_major_locator(plt.MultipleLocator(real_world_multiplier))
    ax.yaxis.set_major_locator(plt.MultipleLocator(real_world_multiplier))
    ax.zaxis.set_major_locator(plt.MultipleLocator(real_world_multiplierZ))
    ax.grid()

    # (px, py) = 3*position[0], 3*position[1]
    # ax.fill([px, px, px + 1, px + 1], [py, py + 1, py + 1, py], color='green')

    for z in range(graph.height):
        for y in range(graph.width):
            for x in range(graph.length):
                if (x, y, z) in g_vals.keys():
                    label_g = ax.text(real_world_multiplier * x + 0.4, real_world_multiplier * y + 0.75,
                                      real_world_multiplierZ * z, str(np.round(g_vals[(x, y, z)], 2)))
                else:
                    label_g = ax.text(real_world_multiplier * x + 0.4, real_world_multiplier * y + 0.75,
                                      real_world_multiplierZ * z, '\u221e')
                if (x, y) in rhs_vals.keys():
                    label_rhs = ax.text(real_world_multiplier * x + 0.4, real_world_multiplier * y + 0.25,
                                        real_world_multiplierZ * z, str(np.round(rhs_vals[(x, y)], 2)))
                else:
                    label_rhs = ax.text(real_world_multiplier * x + 0.4, real_world_multiplier * y + 0.25,
                                        real_world_multiplierZ * z, '1')



# def reconstruct_path(came_from, start, goal):
#     """Reconstruct a shortest path from a dictionary of back-pointers"""
#     current = goal
#     path = [current]
#     while current != start:
#         current = came_from[current]
#         path.append(current)
#     path.append(start)  # optional
#     path.reverse()  # optional
#     return path


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

    g = adaptive_grid3.SquareGrid3(length, width, height)  # Create a Grid that is equivalent to the input grid
    start, end = None, None

    layers = [l.strip() for l in string.split('L') if l.strip()]  # Cuts string into layers
    for layer in range(len(layers)):
        lines = [l2.strip() for l2 in layers[layer].split('\n') if l2.strip()]

        for row, line in enumerate(lines):  # Get the current row and the current line of the grid
            for col, char in enumerate(line):  # Get the current column and the current character of the grid

                if char == adaptive_grid3.WALL:  # If the character is the same as a defined wall character,
                    g.walls.add((col, row, layer))  # add its position to the wall set

                if char == 'o':  # If the character corresponds to the start character,
                    start = (col, row, layer)  # define the start position

                if char == 'X':  # If the character corresponds to the end character,
                    end = (col, row, layer)  # define the end position

    # Make sure that start and end exist
    assert start is not None
    assert end is not None
    return g, start, end  # return the grid object, start and end

# def heuristic(a, b):
#     (x1, y1) = a
#     (x2, y2) = b
#     return abs(x1 - x2) + abs(y1 - y2)
