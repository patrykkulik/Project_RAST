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
