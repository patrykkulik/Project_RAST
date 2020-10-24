import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


mtgarr = [0.01256999999999997, 0.026362999999999914, 5.257356, 0.013803000000000232, 0.019599000000000366, 0.04029100000000074, 1.2149650000000003, 5.098242, 0.7876089999999998, 30.491931, 24.613912999999997, 4.525576999999998, 5.013823000000002, 1.4128630000000015, 10.877919999999989, 2.263709999999989, 24.59171400000001, 1.0100419999999986, 1.1686770000000024, 0.9433169999999933, 1.0870089999999948, 1.091037, 4.136194000000003, 1.378805000000014, 0.7900399999999905, 3.509307000000007, 11.439836999999983, 18.829036000000002, 4.569871000000006, 0.6837069999999983, 5.225278000000003, 1.9967129999999997, 36.065268, 0.6589690000000132, 1.2477669999999819, 0.5868449999999825, 0.7615969999999948, 0.817245999999983, 1.0024680000000217, 1.0401069999999777, 0.870535000000018, 1.0060299999999813, 0.7824699999999893, 0.379587000000015, 0.44181699999998614, 0.46035000000000537, 0.3969020000000114, 0.47778500000001145, 0.04770199999998681, 9.170637999999997, 0.04568900000001008, 0.6487479999999834, 0.6834120000000041, 0.06814900000000534, 3.979672999999991, 0.04532699999998613, 0.04570000000001073, 0.046518000000020265, 0.04917599999998856, 0.05181299999998146, 0.04680200000001378, 0.039573999999987564, 2.299832000000009, 0.04399200000000292, 0.04813500000000204, 1.0090490000000045, 0.04111700000001406, 0.05229799999997908, 0.04772799999997801, 3.813411000000002, 0.053842000000003054, 0.05025600000001873, 0.046371999999990976, 6.544769000000002, 0.03729199999997945, 1.5503390000000081, 0.04039499999998952, 97.42801300000002, 1.3831220000000144, 0.15916499999997313, 0.16792700000002014, 0.19340500000004113, 0.2256720000000314, 0.2357820000000288, 0.16132499999997663, 0.16180700000001025, 0.10544700000002649]
waypoints = [(3, 4, 1, 180.0), (3, 4, 1, -135.0), (3, 4, 1, 180.0), (2, 4, 1, 180.0), (1.5, 4.05, 1.21, 180.0), (1.5, 4.05, 1.21, 0.0), (1.5, 4.05, 1.21, -135.0), (1.5, 4.05, 1.21, -90.0), (1.5, 3.05, 1.21, -90.0), (1.5, 2.125, 1.0, -90.0), (1.5, 2.0, 1.0, -90.0), (1.5, 1.875, 1.0, -90.0), (1.5, 1.875, 1.0, -45.0), (1.625, 1.625, 1.0, -45.0), (1.625, 1.625, 1.0, -90.0), (1.625, 1.5, 1.0, -90.0), (1.625, 1.5, 1.0, 90.0), (1.625, 1.625, 1.0, 90.0), (1.625, 1.625, 1.0, 135.0), (1.5, 1.875, 1.0, 135.0), (1.5, 1.875, 1.0, 90.0), (1.5, 2.0, 1.0, 90.0), (1.5, 2.125, 1.0, 90.0), (1.5, 2.125, 1.0, 180.0), (1.0, 2.25, 1.0, 180.0), (1.0, 2.25, 1.0, -90.0), (1.0, 2.0, 1.0, -90.0), (1.0, 1.875, 1.0, -90.0), (1.0, 1.875, 1.0, -180.0), (0.25, 1.75, 1.0, -180.0), (0.25, 1.75, 1.0, -90.0), (0.25, 1.0, 1.0, -90.0), (0.25, 1.0, 1.0, 45.0), (0.375, 1.125, 1.0, 45.0), (0.375, 1.125, 1.0, 90.0), (0.5, 2.0, 1.0, 90.0), (0.5, 2.0, 1.0, 45.0), (0.75, 2.125, 1.0, 45.0), (1.0, 2.25, 1.0, 45.0), (1.0, 2.25, 1.0, 90.0), (1.0, 2.375, 1.0, 90.0), (1.0, 2.375, 1.0, 45.0), (1.5, 3.05, 1.21, 45.0), (1.5, 3.05, 1.21, 90.0), (1.5, 4.05, 1.21, 90.0), (1.5, 4.05, 1.21, -180.0), (1.0, 4.0, 1.0, -180.0), (1.0, 4.0, 1.0, 180.0), (0.375, 4.0, 1.0, 180.0), (0.25, 4.0, 1.5, 180.0), (0.25, 4.0, 1.5, -135.0), (0.125, 3.875, 2.0, -135.0), (0.125, 3.875, 2.0, 180.0), (0.0, 3.875, 2.5, 180.0), (0.0, 3.875, 2.5, 45.0), (0.25, 4.0, 3.0, 45.0), (0.25, 4.0, 3.0, 0.0), (0.375, 4.0, 3.0, 0.0), (0.5, 4.0, 3.0, 0.0), (1.0, 4.0, 3.0, 0.0), (1.5, 4.0, 3.0, 0.0), (1.625, 4.0, 3.0, 0.0), (2.0, 4.0, 3.0, 0.0), (2.0, 4.0, 3.0, -45.0), (3.0, 3.375, 3.0, -45.0), (3.0, 3.375, 3.0, -90.0), (3.0, 3.0, 3.0, -90.0), (3.0, 2.75, 3.0, -90.0), (3.0, 2.375, 3.0, -90.0), (3.0, 2.0, 3.0, -90.0), (3.0, 1.0, 3.0, -90.0), (3.0, 0.375, 3.0, -90.0), (3.0, 0.25, 3.0, -90.0), (3.0, 0.125, 2.5, -90.0), (3.0, 0.0, 2.0, -90.0), (3.0, 0.0, 2.0, 0.0), (3.0, 0.0, 1.0, 0.0), (3.0, 0.0, 1.0, 90.0), (3.0, 0.25, 1.0, 90.0), (3.0, 1.0, 1.0, 90.0), (3.0, 1.75, 1.0, 90.0), (3.0, 2.0, 1.0, 90.0), (3.0, 2.25, 1.0, 90.0), (3.0, 3.0, 1.0, 90.0), (3.0, 3.25, 1.0, 90.0), (3.0, 3.375, 1.0, 90.0), (3.0, 4.0, 1.0, 90.0)]


xs = []
ys = []
zs = []
zi = []

xs2 = []
ys2 = []
zs2 = []
zi2 = []

xsu = []
ysu = []
zsu = []
ziu = []

xs2u = []
ys2u = []
zs2u = []
zi2u = []

t1 = 25
t2 = 87


# for i in range(t1, t2):
#     a = (waypoints[i])
#
#     if a[2] < 2:
#         xs.append(a[0])
#         ys.append(a[1])
#         zs.append(a[2])
#         zi.append(i)
#         loc = a
#     else:
#         xsu.append(a[0])
#         ysu.append(a[1])
#         zsu.append(a[2])
#         ziu.append(i)
#         locu = a
#
# for i in range(0, t1):
#     a = (waypoints[i])
#
#     if a[2] < 2:
#         xs2.append(a[0])
#         ys2.append(a[1])
#         zs2.append(a[2])
#         zi2.append(i)
#
#     else:
#         xs2u.append(a[0])
#         ys2u.append(a[1])
#         zs2u.append(a[2])
#         zi2u.append(i)

for i in range(t1, t2):
    a = (waypoints[i])
    
    xs.append(a[0])
    ys.append(a[1])
    zs.append(a[2])
    zi.append(i)
    loc = a


for i in range(0, t1):
    a = (waypoints[i])
    
    xs2.append(a[0])
    ys2.append(a[1])
    zs2.append(a[2])
    zi2.append(i)


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(mtgarr)
ax.set_xlabel('Steps')
ax.set_ylabel('Time taken (s)')

# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111)
# ax2.set_xlim(0, 5)
# ax2.set_ylim(0, 5)
#
# ax2.plot(3, 4, 's', color='gray')
# ax2.plot(xs2, ys2, color='gray')
# ax2.plot(xs, ys, 'r')
# ax2.plot(waypoints[t2 - 1][0], waypoints[t2 - 1][1], 'ro')

# fig3 = plt.figure()
# ax3 = fig3.add_subplot(111)
# ax3.plot(zi2, zs2, color='gray')
# ax3.plot(zi, zs, 'k')

# fig4 = plt.figure()
# ax4 = fig4.add_subplot(111)
# ax4.set_xlim(0, 5)
# ax4.set_ylim(0, 5)
#
# ax4.plot(3, 4, 's', color='gray')
# ax4.plot(xs2u, ys2u, color='gray')
# ax4.plot(xsu, ysu, 'r')
# ax4.plot(waypoints[t2 - 1][0], waypoints[t2 - 1][1], 'ro')

fig5 = plt.figure()
ax5 = fig5.add_subplot(111, projection='3d', proj_type = 'ortho')
ax5.set_xlim(0, 4)
ax5.set_ylim(0, 5)
ax5.set_zlim(0, 5)
ax5.view_init(19,73)

ax5.plot([3], [4], [1], 's', color='gray')
ax5.plot(xs2, ys2, zs2, color='gray')
ax5.plot(xs, ys, zs, 'r')
ax5.plot([waypoints[t2 - 1][0]], [waypoints[t2 - 1][1]], [waypoints[t2 - 1][2]], 'ro')

walls = []
walls2 = []

multiplier = 2
y1 = np.arange(0, 7.3, 0.2)
y2 = np.arange(6.6, 7.3, 0.2)
y3 = np.arange(0, 1.2, 0.2)
y4 = np.arange(1, 4.2, 0.2)
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
        walls.append((0 / multiplier, y1[i] / multiplier, z))
        walls.append((3.8 / multiplier, y1[i] / multiplier, z))
    for i in range(len(y2)):
        walls.append((1 / multiplier, y2[i] / multiplier, z))
    for i in range(len(y3)):
        walls.append((1.8 / multiplier, y3[i] / multiplier, z))
    for i in range(len(y4)):
        walls.append((2.8 / multiplier, y4[i] / multiplier, z))
    for i in range(len(x1)):
        walls.append((x1[i] / multiplier, 0/ multiplier, z))
        walls.append((x1[i] / multiplier, 7.1 / multiplier, z))
    for i in range(len(x2)):
        walls.append((x2[i] / multiplier, 4 / multiplier, z))
    for i in range(len(x3)):
        walls.append((x3[i] / multiplier, 5.5 / multiplier, z))
    for i in range(len(x5)):
        walls.append((x5[i] / multiplier, 4 / multiplier, z))
z = 2
flr = []
for x in x1:
    for y in np.arange(0, 1.2, 0.2):
        y = np.round(y, 4)
        flr.append((x / multiplier, y / multiplier, z))
for x in np.arange(0, 3, 0.2):
    for y in np.arange(1, 4.2, 0.2):
        x = np.round(x, 4)
        y = np.round(y, 4)
        flr.append((x / multiplier, y / multiplier, z))
for x in x1:
    for y in np.arange(4, 7.3, 0.2):
        y = np.round(y, 4)
        flr.append((x / multiplier, y / multiplier, z))

# Second floor
y1 = np.arange(0, 7.3, 0.2)
y2 = np.arange(0, 3.6, 0.2)
y3 = np.arange(4.5, 5.3, 0.2)
y4 = np.arange(5.1, 7.3, 0.2)
x1 = np.arange(0, 4, 0.2)
x2 = np.arange(0, 1.8, 0.2)
x3 = np.arange(3.6, 4, 0.2)
x5 = np.arange(0, 2, 0.2)
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
        walls2.append((0 / multiplier, y1[i] / multiplier, z))
        walls2.append((3.8 / multiplier, y1[i] / multiplier, z))
    for i in range(len(y2)):
        walls2.append((2.8 / multiplier, y2[i] / multiplier, z))
    for i in range(len(y3)):
        walls2.append((1.6 / multiplier, y3[i] / multiplier, z))
    for i in range(len(y4)):
        walls2.append((2.6 / multiplier, y4[i] / multiplier, z))
    for i in range(len(x1)):
        walls2.append((x1[i] / multiplier, 0/ multiplier, z))
        walls2.append((x1[i] / multiplier, 7.1 / multiplier, z))
    for i in range(len(x2)):
        walls2.append((x2[i] / multiplier, 5.1 / multiplier, z))
    for i in range(len(x3)):
        walls2.append((x3[i] / multiplier, 5.1 / multiplier, z))
    for i in range(len(x5)):
        walls2.append((x5[i] / multiplier, 3.4 / multiplier, z))
z = 4
cel = []
for x in x1:
    for y in y1:
        cel.append((x / multiplier, y / multiplier, z))

for w in walls:
    ax5.plot([w[0]], [w[1]], [w[2]], '.', color = 'black', alpha = 0.05)

for f in flr:
    ax5.plot([f[0]], [f[1]], [f[2]], '.', color = 'black', alpha = 0.5)

for w in walls2:
    ax5.plot([w[0]], [w[1]], [w[2]], '.', color = 'black', alpha = 0.05)
    
print(len(waypoints))
    

    


plt.pause(100)