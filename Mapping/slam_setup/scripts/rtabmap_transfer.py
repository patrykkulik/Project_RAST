import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from pyquaternion import Quaternion
import tf
import sys

vehicle_type = sys.argv[1]
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'

def vins_callback(data):
    local_pose.header.stamp = data.header.stamp
    local_pose.pose = data.pose.pose
    
rospy.init_node('vins_transfer')
rospy.Subscriber("/iris_0/stereo_camera/odom", Odometry, vins_callback)
position_pub = rospy.Publisher(vehicle_type+"_0/mavros/vision_pose/pose", PoseStamped, queue_size=2)
rate = rospy.Rate(20) 

while True:
    local_pose.header.stamp = rospy.Time.now()
    position_pub.publish(local_pose) 
    rate.sleep()
  
