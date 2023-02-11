#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class OdomData:
    
    def __init__(self):

        #Define subscriber to odom
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.robot_pose)

        #Define publisher to odom
        self.odom_pub = rospy.Publisher("/odom2", String, queue_size=10)

        #Define subscriber to occupancy grid
        self.occupancy_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.robot_pose)

        #Define publisher to occupancy grid
        self.occupancy_pub = rospy.Publisher("/occupancy_data", String, queue_size=10)


    #Callback function
    def robot_pose(self, pose_msg):

        #Robot position
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        self.z = pose_msg.pose.pose.orientation.z

        print(self.x, self.y, self.z)

    #Callback function to
    def get_local_costmap(self, costmap_msg):

        #Receive local costmap data as 1D array
        self.local_costmap = costmap_msg.data

if __name__ == "__main__":

    #make node 
    rospy.init_node('nav_msg_odometry', anonymous=True)

    try:
        OdomData()
        rospy.spin()
    except rospy.ROSInterruptException:
    	pass
    	
    
