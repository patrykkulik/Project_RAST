#
# mr-dfs.py
#
# Created by Geoffrey Sheir on 21/05/2020
#
#

import cases 
from rob import robot
from plotTools import plotter
import matplotlib
import matplotlib.pyplot as plt

### SET PARAMETERS HERE ###
k = 1 # number of robots
case = cases.case10(k)
prob = case.gr

# initialise robots
robots = []
for rID in range(k):
    robots.append(robot(rID))

# tell robots what the problem is
# CHANGE
for rID in range(k):
    robots[rID].setProb(prob)
    
#%% run algorithm

# initial step for all robots
for rID in range(k):
    print("rob{} starting at v{}".format(rID,robots[rID].vNow))
    E = len(prob.v[0].e)
    robots[rID].updatePos(0, rID%E)

endExp = False
while endExp == False:
    

    for rID in range(k):
        robots[rID].mrdfs()
        
    algoFinished = True
    for i in range(len(prob.v[0].e)):
        if prob.v[0].e[i].finished == False:
            algoFinished = False
            break
        
    allHome = True
    for rID in range(k):
        if robots[rID].vNow != 0:
            allHome = False
            break
    
    if (algoFinished == True) and (allHome == True):
        print("Exploration complete")
        endExp = True
        break

# create animation

anim = plotter(robots,case)
anim.playAnim()

import time
time.sleep(100)