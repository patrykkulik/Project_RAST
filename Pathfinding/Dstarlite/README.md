# Greedy D* Lite Navigation Algorithm

![Final_navigation](https://user-images.githubusercontent.com/31861043/85065008-59f1e780-b1a4-11ea-8733-c1e67f78df5c.gif)

The folders contain a series of implementations of the Greedy D* Lite algorithm as well as initial goal-seeking versions of the algorithm.


Goal seeking:
* `Dstarlite_2D`: 2D, fixed-size grid
* `Dstarlite_field`: 2D, fixed-size, able to move on non-grid nodes

Exploratory (Greedy):
* `Greedy`: 2D, fixed-size grid
* `3D`: 3D, fixed-size grid
* `Adaptive_grid`: 2D, adaptive grid
* `adaptive3`: 3D, adaptive grid
* `integration`: 3D, fully integrated version with ROS subscriber features

The `integration` folder contains the final version of the algorithm. The `Test_cases.py` file contains various house layouts for which the algorithm was tested. 

##### Features:
* Dynamic map building based on SLAM data
* Dynamic path-planning based on available data
* Static obstacle avoidance

##### Requirements:
* Python 2.7 and later
* Numpy
* Matplotlib
* Refer to the `README.md` in the `integration` folder for specific requirements. 

#### Credit:
* Original paper: https://www.cs.cmu.edu/~maxim/files/dlite_tro05.pdf
* Code was build upon the work of @samdjstephens: https://github.com/samdjstephens/pydstarlite
