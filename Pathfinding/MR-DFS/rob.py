#
# rob.py
#
# Created by Geoffrey Sheir on 20/05/2020
#
#
import graphTools

class robot:
    def __init__(self, rid):
        self.rID = rid
        self.vNow = 0 # current vertex
        self.eNow = 0 # edge just used
        self.moves = 0
        self.posHist = [0]
        
        self.record = graphTools.graph()
        self.coords = (0,0,0) # CHECK if works with IMU
        self.k = 1
        
    def setProb(self,case):
        self.prob = case
        
    def updatePos(self, vnowID, eleaveID):
        self.moves += 1
        self.prob.v[vnowID].e[eleaveID].exits[self.rID] += 1
        
        # set last exit
        for i in range(len(self.prob.v[vnowID].e)):
            self.prob.v[vnowID].e[i].lastExit[self.rID] = False 
        self.prob.v[vnowID].e[eleaveID].lastExit[self.rID] = True
        self.prob.v[vnowID].visited[self.rID] = True
        
        # entry to other vertex
        dest = self.prob.graphMap[(vnowID,eleaveID)]
        print("rob{} move {} (v{},e{}) to (v{},e{})".format(self.rID,self.moves,self.vNow,self.eNow,dest[0],dest[1]))
        self.vNow = dest[0];
        self.eNow = dest[1];
        self.prob.v[self.vNow].e[self.eNow].entries[self.rID] += 1  
        self.posHist.append(self.vNow)
        
    def updatePosNoExit(self, vnowID, eleaveID):
        self.moves += 1
        self.prob.v[vnowID].e[eleaveID].exits[self.rID] += 1
        
        # # set last exit
        # for i in range(len(self.prob.v[vnowID].e)):
        #     self.prob.v[vnowID].e[i].lastExit[self.rID] = False 
        # self.prob.v[vnowID].e[eleaveID].lastExit[self.rID] = True
        # self.prob.v[vnowID].visited[self.rID] = True
        
        # entry to other vertex
        dest = self.prob.graphMap[(vnowID,eleaveID)]
        print("rob{} move {} (v{},e{}) to (v{},e{})".format(self.rID,self.moves,self.vNow,self.eNow,dest[0],dest[1]))
        self.vNow = dest[0];
        self.eNow = dest[1];
        self.prob.v[self.vNow].e[self.eNow].entries[self.rID] += 1  
        self.posHist.append(self.vNow)
        
    def setEdgeFinished(self,vID,eID): # both ends of the edge
        self.prob.v[vID].e[eID].finished = True
        print("rob{} marked (v{},e{}) finished".format(self.rID,vID,eID))
        other = self.prob.graphMap[(vID,eID)]
        self.prob.v[other[0]].e[other[1]].finished = True
        print("rob{} marked (v{},e{}) finished".format(self.rID,other[0],other[1]))
        
    def setK(self, k):
        self.k = k
        
    def addToRec(self): # add one blank vertex to record
        self.record.addVertex(1, k)
    
    def queryRec(self,coords): # ADD
        return something # return a vertex, or if none found then return false maybe?
    
        
# %% algorithm implementation

    def mrdfs(self): 
        rob = self.rID
        vi = self.prob.v[self.vNow]
        ei = self.prob.v[self.vNow].e[self.eNow]
        
        # 2
        if (vi.visited[rob] == True) and (ei.lastExit[rob] == False):
            self.setEdgeFinished(vi.vID, ei.eID)
            
            # additional finishing criterion
            algoFinished = True
            for i in range(len(self.prob.v[0].e)):
                if self.prob.v[0].e[i].finished == False:
                    algoFinished = False
                    break
            
            if algoFinished == True:
                endExp = True
                return
            
            self.updatePosNoExit(vi.vID,ei.eID)
        # 4
        else:
            # 6
            if (vi.visited[rob] == False):
                ei.setFirstIn(rob)
            # 8   
            else:
                self.setEdgeFinished(vi.vID, ei.eID)
                
            prefAssigned = False
            prefFirstInAssigned = False
            
            # 12-15
            for i in range(len(vi.e)):
                if (vi.e[i].finished == False): 
                    
                    if ((True in vi.e[i].firstIn) == False):
                        if prefAssigned == False:
                            pref = vi.e[i]
                            prefTotal = sum(pref.entries) + sum(pref.exits)
                            prefAssigned = True
                            
                        else:                    
                            eiTotal = vi.e[i].entries[rob] + vi.e[i].exits[rob]
                        
                            if eiTotal < prefTotal:
                                pref = vi.e[i]
                
                if vi.e[i].firstIn[rob] == True:
                    prefFirstIn = vi.e[i]
                    prefFirstInAssigned = True

            if prefAssigned == True:
                self.updatePos(vi.vID, pref.eID)
            elif prefFirstInAssigned == True:
                self.updatePos(vi.vID, prefFirstIn.eID)
            else:
                print("rob{} move {} stay at (v{},e{})".format(self.rID,self.moves,vi.vID,ei.eID))
                self.posHist.append(self.vNow)
            
            # immediately set as finished if so
            vi = self.prob.v[self.vNow]
            ei = self.prob.v[self.vNow].e[self.eNow]
            if (vi.visited[rob] == True) and (ei.lastExit[rob] == True):
                self.setEdgeFinished(vi.vID, ei.eID)
                
                   