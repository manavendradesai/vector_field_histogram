#!/usr/bin/env python3
import rospy
import numpy as np

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64, Int8
from std_msgs.msg import Float64MultiArray, Int8MultiArray


class OdomData:
    
    def __init__(self):

	    #self.local_costmap

        #Define subscriber to odom
        self.odom_sub = rospy.Subscriber("/odom",Odometry, self.robot_pose)

        #Define publisher to odom
        self.odom_pub = rospy.Publisher("/odom2", Float64MultiArray, queue_size=10)
        
        #Define subscriber to occupancy grid
        self.occupancy_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.get_local_costmap)

        #Define publisher to occupancy grid
        self.occupancy_pub = rospy.Publisher("/occupancy_data", Int8MultiArray, queue_size=1)
        
        #Define publisher for VFH waypoint marker
        self.marker_pub = rospy.Publisher("/visualization_marker",Marker,queue_size=2)

        #Define publisher for goal marker
        self.goal_pub = rospy.Publisher("/goal_marker",Marker,queue_size=2)


    #Callback function
    def robot_pose(self, pose_msg):

        #Robot position
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        self.z = pose_msg.pose.pose.orientation.z

        pos = Float64MultiArray()
        pos.data = np.array([self.x, self.y, self.z])

        self.pub_marker(self.x, self.y, self.z)
        self.goal_marker(self.x+1, self.y+1, self.z)
        self.odom_pub.publish(pos)

        #print(self.x, self.y, self.z)

    #Callback function to
    def get_local_costmap(self, costmap_msg):

        occGrid = costmap_msg.data

        self.local_costmap = Int8MultiArray()

        #Receive local costmap data as 1D array
        self.local_costmap.data = occGrid
        
        #print(self.local_costmap)
        
        self.occupancy_pub.publish(self.local_costmap)

    #A function that publishes a marker at the VFH waypoint
    def pub_marker(self,wpx,wpy,wppsi):

        #Define marker
        self.marker = Marker()

        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.type = 0
        self.marker.id = 0

        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 0
        self.marker.color.g = 0
        self.marker.color.b = 1
        self.marker.color.a = 1

        psiR = wppsi

        self.marker.pose.position.x = wpx
        self.marker.pose.position.y = wpy
        self.marker.pose.position.z = 0.1
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = np.sin(psiR/2)
        self.marker.pose.orientation.w = np.cos(psiR/2)

        #Publish marker
        self.marker_pub.publish(self.marker)

    #A function that publishes a marker at the goal
    def goal_marker(self,wpx,wpy,wppsi):

        #Define marker
        self.marker = Marker()

        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.type = 0
        self.marker.id = 0

        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 0
        self.marker.color.g = 1
        self.marker.color.b = 0
        self.marker.color.a = 1

        psiR = wppsi

        self.marker.pose.position.x = wpx
        self.marker.pose.position.y = wpy
        self.marker.pose.position.z = 0.2
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = np.sin(psiR/2)
        self.marker.pose.orientation.w = np.cos(psiR/2)

        #Publish marker
        self.goal_pub.publish(self.marker)


if __name__ == "__main__":

    #make node 
    rospy.init_node('nav_msg_odometry', anonymous=True)

    try:
        OdomData()
        rospy.spin()
    except rospy.ROSInterruptException:
    	pass

""" https://www.programcreek.com/python/example/95997/nav_msgs.msg.OccupancyGrid """



