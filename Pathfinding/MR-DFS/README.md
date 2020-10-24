# MR-DFS

Python implementation of the Multi-Robot Depth-First Search algorithm. 
![search](https://github.com/mirkokovac/GDP2020/blob/master/Pathfinding/MR-DFS/gif/case9.3.gif?raw=true)

##### Requirements:
* Python 3.7
* Numpy
* Matplotlib
* NetworkX `pip install networkx` or through Anaconda

Note: in Spyder Preferences, under IPython Console -> Graphics -> Graphics Backend, select Automatic. Figures open in a separate window. 

##### Description:
* `rob.py` contains main MR-DFS implementation within `robot` class
* `graphTools.py` contains graph components to build test cases
* `cases.py` contains test cases 
* `plotTools.py` provides the animation function for visualisation
* `mr-dfs.py` main routine

To run: Set number of robots and test case in `mr-dfs.py`, then run.  

#### Credit:
* Original paper: https://ieeexplore.ieee.org/document/5739538
