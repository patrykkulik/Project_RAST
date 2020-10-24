#
# cases.py
#
# Created by Geoffrey Sheir on 20/05/2020
#
#

import graphTools as gt
import networkx as nx

class case1: # 2 nodes with one edge between them
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(2,k)
        
        self.gr.v[0].addEdge(1,k)
        self.gr.v[1].addEdge(1,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0),
            (1,0) : (0,0)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1])
        self.G.add_edge(0,1)
        self.pos = {
               0:(0,0),
               1:(1,0)
               }
    

        
    
class case2: # 1 node leads to 2 nodes
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(3,k)
        
        self.gr.v[0].addEdge(2,k)
        self.gr.v[1].addEdge(1,k)
        self.gr.v[2].addEdge(1,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (0,1) : (2,0), (2,0) : (0,1)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2])
        self.G.add_edge(0,1)
        self.G.add_edge(0,2)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(1,-1)
               }

class case3: # triangle
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(3,k)
        
        self.gr.v[0].addEdge(2,k)
        self.gr.v[1].addEdge(2,k)
        self.gr.v[2].addEdge(2,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (0,1) : (2,1), (2,1) : (0,1),
            (1,1) : (2,0), (2,0) : (1,1)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2])
        self.G.add_edge(0,1)
        self.G.add_edge(0,2)
        self.G.add_edge(1,2)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(1,-1)
               }

class case4: # 1 node leads to 2 nodes, one leg longer
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(4,k)
        
        self.gr.v[0].addEdge(2,k)
        self.gr.v[1].addEdge(1,k)
        self.gr.v[2].addEdge(2,k)
        self.gr.v[3].addEdge(1,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (0,1) : (2,0), (2,0) : (0,1),
            (2,1) : (3,0), (3,0) : (2,1)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3])
        self.G.add_edge(0,1)
        self.G.add_edge(0,2)
        self.G.add_edge(2,3)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(1,-1),
               3:(2,-1)
               }
        
        
class case5: # triangle with extra branch
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(4,k)
        
        self.gr.v[0].addEdge(2,k)
        self.gr.v[1].addEdge(2,k)
        self.gr.v[2].addEdge(3,k)
        self.gr.v[3].addEdge(1,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (0,1) : (2,0), (2,0) : (0,1),
            (2,1) : (3,0), (3,0) : (2,1),
            (1,1) : (2,2), (2,2) : (1,1)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3])
        self.G.add_edge(0,1)
        self.G.add_edge(0,2)
        self.G.add_edge(1,2)
        self.G.add_edge(2,3)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(1,-1),
               3:(2,-1)
               }
        
        
class case6: # 6 noded rect
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(6,k)
        
        self.gr.v[0].addEdge(3,k)
        self.gr.v[1].addEdge(3,k)
        self.gr.v[2].addEdge(2,k)
        self.gr.v[3].addEdge(2,k)
        self.gr.v[4].addEdge(4,k)
        self.gr.v[5].addEdge(2,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (0,1) : (4,0), (4,0) : (0,1),
            (0,2) : (3,0), (3,0) : (0,2),
            (1,1) : (4,1), (4,1) : (1,1),
            (1,2) : (2,0), (2,0) : (1,2),
            (2,1) : (5,0), (5,0) : (2,1),
            (3,1) : (4,3), (4,3) : (3,1),
            (5,1) : (4,2), (4,2) : (5,1),
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3,4,5])
        self.G.add_edge(0,1)
        self.G.add_edge(0,4)
        self.G.add_edge(1,2)
        self.G.add_edge(2,5)
        self.G.add_edge(4,5)
        self.G.add_edge(3,4)
        self.G.add_edge(0,3)
        self.G.add_edge(1,4)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(2,0),
               3:(0,-1),
               4:(1,-1),
               5:(2,-1),
               }
        
class case7: # 1 node leads to 3 nodes
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(4,k)
        
        self.gr.v[0].addEdge(3,k)
        self.gr.v[1].addEdge(1,k)
        self.gr.v[2].addEdge(1,k)
        self.gr.v[3].addEdge(1,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (0,1) : (2,0), (2,0) : (0,1),
            (0,2) : (3,0), (3,0) : (0,2)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3])
        self.G.add_edge(0,1)
        self.G.add_edge(0,2)
        self.G.add_edge(0,3)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(1,-1),
               3:(0,-1)
               }
        
class case8: # 3 level tree
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(6,k)
        
        self.gr.v[0].addEdge(3,k)
        self.gr.v[1].addEdge(3,k)
        self.gr.v[2].addEdge(1,k)
        self.gr.v[3].addEdge(1,k)
        self.gr.v[4].addEdge(1,k)
        self.gr.v[5].addEdge(1,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (0,1) : (2,0), (2,0) : (0,1),
            (0,2) : (3,0), (3,0) : (0,2),
            (1,1) : (4,0), (4,0) : (1,1),
            (1,2) : (5,0), (5,0) : (1,2)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3,4,5])
        self.G.add_edge(0,1)
        self.G.add_edge(0,2)
        self.G.add_edge(0,3)
        self.G.add_edge(1,4)
        self.G.add_edge(1,5)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(1,-1),
               3:(0,-1),
               4:(2,0),
               5:(2,-1)
               }
        
class case9: # big stress test
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(13,k)
        
        self.gr.v[0].addEdge(2,k)
        self.gr.v[1].addEdge(2,k)
        self.gr.v[2].addEdge(3,k)
        self.gr.v[3].addEdge(2,k)
        self.gr.v[4].addEdge(1,k)
        self.gr.v[5].addEdge(3,k)
        self.gr.v[6].addEdge(3,k)
        self.gr.v[7].addEdge(3,k)
        self.gr.v[8].addEdge(1,k)
        self.gr.v[9].addEdge(1,k)
        self.gr.v[10].addEdge(2,k)
        self.gr.v[11].addEdge(2,k)
        self.gr.v[12].addEdge(1,k)
        
        
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (1,1) : (2,0), (2,0) : (1,1),
            (2,1) : (3,0), (3,0) : (2,1),
            (3,1) : (4,0), (4,0) : (3,1),
            (0,1) : (5,0), (5,0) : (0,1),
            (5,1) : (6,0), (6,0) : (5,1),
            (6,1) : (7,0), (7,0) : (6,1),
            (7,1) : (2,2), (2,2) : (7,1),
            (7,2) : (8,0), (8,0) : (7,2),
            (5,2) : (9,0), (9,0) : (5,2),
            (6,2) : (10,0), (10,0) : (6,2),
            (10,1) : (11,0), (11,0) : (10,1),
            (11,1) : (12,0), (12,0) : (11,1)            
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3,4,5,6,7,8,9,10,11,12])
        self.G.add_edge(0,1)
        self.G.add_edge(1,2)
        self.G.add_edge(2,3)
        self.G.add_edge(3,4)
        self.G.add_edge(0,5)
        self.G.add_edge(5,6)
        self.G.add_edge(6,7)
        self.G.add_edge(2,7)
        self.G.add_edge(7,8)
        self.G.add_edge(5,9)
        self.G.add_edge(6,10)
        self.G.add_edge(10,11)
        self.G.add_edge(11,12)
        self.pos = {
               0:(0,0),
               1:(1,0),
               2:(2,0),
               3:(3,0),
               4:(4,0),
               5:(0,-1),
               6:(1,-1),
               7:(2,-1),
               8:(3,-1),
               9:(0,-2),
               10:(1,-2),
               11:(2,-2),
               12:(3,-2)
               }
        
class case10: # T shape
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(7,k)
        
        self.gr.v[0].addEdge(1,k)
        self.gr.v[1].addEdge(3,k)
        self.gr.v[2].addEdge(3,k)
        self.gr.v[3].addEdge(3,k)
        self.gr.v[4].addEdge(1,k)
        self.gr.v[5].addEdge(4,k)
        self.gr.v[6].addEdge(1,k)
        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (1,1) : (2,0), (2,0) : (1,1),
            (2,1) : (3,0), (3,0) : (2,1),
            (3,1) : (4,0), (4,0) : (3,1),
            (1,2) : (5,0), (5,0) : (1,2),
            (2,2) : (5,1), (5,1) : (2,2),
            (3,2) : (5,2), (5,2) : (3,2),
            (5,3) : (6,0), (6,0) : (5,3)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3,4,5,6])
        self.G.add_edge(0,1)
        self.G.add_edge(1,2)
        self.G.add_edge(2,3)
        self.G.add_edge(3,4)
        self.G.add_edge(1,5)
        self.G.add_edge(2,5)
        self.G.add_edge(3,5)
        self.G.add_edge(6,5)
        self.pos = {
            0:(0,0),
            1:(1,0),
            2:(2,0),
            3:(3,0),
            4:(4,0),
            5:(2,-1),
            6:(2,-2),
               }
        
class case11: # _∆
    def __init__(self, k):
        self.gr = gt.graph()
        
        self.gr.addVertex(4,k)
        
        self.gr.v[0].addEdge(1,k)
        self.gr.v[1].addEdge(3,k)
        self.gr.v[2].addEdge(2,k)
        self.gr.v[3].addEdge(2,k)

        
        # Vertex connections organised as tuple-to-tuple dict (directed!) (v,e)
        self.gr.graphMap = {
            (0,0) : (1,0), (1,0) : (0,0),
            (1,1) : (2,0), (2,0) : (1,1),
            (2,1) : (3,0), (3,0) : (2,1),
            (3,1) : (1,2), (1,2) : (3,1)
            }
        
        # Plotting tools
        self.G = nx.Graph()
        self.G.add_nodes_from([0,1,2,3])
        self.G.add_edge(0,1)
        self.G.add_edge(1,2)
        self.G.add_edge(2,3)
        self.G.add_edge(3,1)

        self.pos = {
            0:(0,0),
            1:(1,0),
            2:(2,0),
            3:(1,-1)
               }