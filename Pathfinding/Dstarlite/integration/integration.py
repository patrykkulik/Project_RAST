#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2
import rospy
import ros_numpy
import numpy as np
from utility3 import initDomain

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from adaptive3 import adaptive3
import time
# import sys
# from droneStatic/srv import * # replace with package name

class drone(object):
    def __init__(self, length, width, height, start):
        print("\n drone __init__()")
        self.np_points = None
        
        # initiate algorithm
        GRAPH, START, END = initDomain(length, width, height, start)
        self.dstar = adaptive3(GRAPH, START, None)
        self.ax = self.dstar.initialise()
        self.waypoint = None
        
        self.tftimearr = []
        self.mtgarr = []

        
    def callback(self, data):
        print("\n drone callback()")
        (px,py,pz) = (self.dstar.position[0]*self.dstar.real_world_multiplier,self.dstar.position[1]*self.dstar.real_world_multiplier,self.dstar.position[2]*self.dstar.real_world_multiplierZ)
        print("pos {} orien {}".format((px,py,pz), self.dstar.orientation))
        
        # converts pointcloud to np array
        pc = ros_numpy.numpify(data)
        height = pc.shape[0]
        width = 1
        self.np_points = np.zeros((height * width, 3), dtype=np.float32)
        self.np_points[:, 0] = np.resize(pc['x'], height * width) / self.dstar.real_world_multiplier
        self.np_points[:, 1] = np.resize(pc['y'], height * width) / self.dstar.real_world_multiplier
        self.np_points[:, 2] = np.resize(pc['z'], height * width) / self.dstar.real_world_multiplierZ

        # if loop to extract pc data when needed
        condition = True
        wayp = None

        if condition:
            tftime = time.clock()
            self.dstar.real_graph.tfSlam(self.dstar.position, self.dstar.orientation, self.np_points)
            self.tftimearr.append(time.clock() - tftime)
            # print("tfSlamtime = {}" .format(self.tftimearr))
            
            
            if self.dstar.position != self.dstar.start or len(self.dstar.seen) != len(self.dstar.graph.available_nodes):
                # while the goal has not been reached
                movetime = time.clock()
                wayp = self.dstar.move_to_goal(self.ax)
                self.mtgarr.append(time.clock() - movetime)
                # print("move_to_goaltime = {}" .format(self.mtgarr))
                
        # publish waypoint
        self.waypoint = (wayp[0]*self.dstar.real_world_multiplier, wayp[1] * self.dstar.real_world_multiplier, wayp[2] * self.dstar.real_world_multiplierZ)
        print("waypoint {}".format(self.waypoint))
        return
    
    def listener(self):
        rospy.init_node('pcl_listener', anonymous=True)  # creates a node
        # rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback)  # subscribes to the ros topic (this line for the Kinect)
        rospy.Subscriber('/rtabmap/cloud_obstacles', PointCloud2, self.callback) # for the actual slam data
        rospy.spin()

def main():
    (length, width, height) = (10,10,3) # in normalised coords
    drone1 = drone(length, width, height, (5,5,0)) # normalised coords
    drone1.listener()
    
    
if __name__ == "__main__":
    main()

    # def getSlam_client(x, y):
    #     rospy.wait_for_service('get_slam')
    #     try:
    #         getSlam = rospy.ServiceProxy('get_slam', getSlam)
    #         resp1 = getSlam(x, y)
    #         return resp1.sum
    #     except rospy.ServiceException, e:
    #         print "Service call failed: %s"%e
    #
    # def usage():
    #     return "%s [x y]"%sys.argv[0]
    #
    # if __name__ == "__main__":
    #     if len(sys.argv) == 3:
    #         x = int(sys.argv[1])
    #         y = int(sys.argv[2])
    #     else:
    #         print usage()
    #         sys.exit(1)
    #     print "Requesting %s+%s"%(x, y)
    #     print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
    
    # publisher/subscriber paradigm