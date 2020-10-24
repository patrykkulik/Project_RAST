#
# graphTools.py
#
# Created by Geoffrey Sheir on 20/05/2020
#
#

class edge:
    def __init__(self, eid, k):
        self.eID = eid # within vertex
        self.firstIn = [False]*k # first entrance for each rob
        self.lastExit = [False]*k # last exit for each rob
        self.entries = [0]*k # count of entries for each rob
        self.exits = [0]*k # count of exits for each rob
        self.finished = False 
        
    def setFirstIn(self,rob):
        self.firstIn[rob] = True
        
        

class vertex:
    def __init__(self, vid, k):
        self.vID = vid
        self.visited = [False]*k # whether any rob has visited
        self.e = []

    def addEdge(self, E, k):
        for i in range(E):
            self.e.append(edge(i, k))
        
        
class graph:
    def __init__(self):
        self.v = []
        self.graphMap = {}
    
    def addVertex(self, V, k):
        for i in range(V):
            self.v.append(vertex(i,k))
            

        