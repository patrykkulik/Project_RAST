#
# plotTools.py
#
# Created by Geoffrey Sheir on 22/05/2020
#
#

import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class plotter:
    def __init__(self, robots,case):
        plt.close("all")
        self.robots = robots
        self.case = case
        self.ann = None
        
        self.fig = plt.figure()
        self.ax = plt.axes()
        self.line = []
        self.point = []
        self.vertex = 0
        
        self.x = []
        self.y = []
        self.linestyles = ['r-','c-','y-','m-']
        self.pointstyles = ['ro','co','yo','mo']
        
        
        for rID in range(len(self.robots)):
            self.line.append(0)
            self.point.append(0)

            self.line[rID], = self.ax.plot([],[],self.linestyles[rID],linewidth = 2-rID*0.2)
            self.point[rID], = self.ax.plot([],[],self.pointstyles[rID],markersize = 20-2*rID)

            
            self.x.append([])
            self.y.append([])
            
            for i in range(len(self.robots[rID].posHist)):
                self.x[rID].append(None)
                self.y[rID].append(None)
            
    def init(self):
        for rID in range(len(self.robots)):
            self.line[rID].set_data([],[])
            self.point[rID].set_data([],[])
        return self.line, self.point,
    
    def animate(self,i):
        for rID in range(len(self.robots)):
            # if (i % len(self.robots[rID].posHist) == 0):
            #     print("clear")
            #     for j in range(len(self.robots[rID].posHist)):
            #         self.x[rID][j] = [None]
            #         self.y[rID][j] = [None]
            
            
                
            
            (xi,yi) = self.case.pos[self.robots[rID].posHist[i]]
            self.x[rID][i] = xi
            self.y[rID][i] = yi
            self.line[rID].set_data(self.x[rID],self.y[rID])
            self.point[rID].set_data(xi,yi)

            # self.ann = self.ax.annotate("rob{}".format(rID),xy = (x,y))

        return self.line, self.point,
    
    def playAnim(self):
        nx.draw(self.case.G, self.case.pos, with_labels = True, node_size = 300, node_color = 'k', font_color = 'w', font_size = 10, font_weight = 'bold', width = 5)
        self.anim = FuncAnimation(self.fig, self.animate, init_func = self.init, frames = len(self.robots[0].posHist), interval = 500, blit = False)
        plt.show

