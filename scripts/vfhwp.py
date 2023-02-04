#! /usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

class VFH:

    def __init__(self):
        #Get parameters
        #Robot diameter
        self.dia = rospy.get_param('/vfh/dia',0.2)
        #VFH weights
        self.wg = rospy.get_param('/vfh/wg',3)
        self.wo = rospy.get_param('/vfh/wo',1)
        self.wd = rospy.get_param('/vfh/wd',1)
        #Lookahead distance
        self.LA = rospy.get_param('/vfh/LA',11)

        #Define subscriber to odom
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.robot_pose)

        # #Define subscriber to local_costmap data
        # self.local_costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap",
        # OccupancyGrid,self.local_costmap)

        #Define publisher for marker
        self.marker_pub = rospy.Publisher("/visualization_marker",Marker,queue_size=2)

    #Receive robot pose
    def robot_pose(self,pose_msg):
        self.X = pose_msg.pose.pose.position.x
        self.Y = pose_msg.pose.pose.position.y
        psiz = pose_msg.pose.pose.orientation.z
        psiw = pose_msg.pose.pose.orientation.w
        self.psi = 2*np.arctan((psiz)/(0.001+psiw))
        print(self.psi)
    
    # #Recieve local costmap data
    # def local_costmap(self,costmap_msg):



    def pub_marker(self):

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

        psiR = np.pi

        self.marker.pose.position.x = 1
        self.marker.pose.position.y = 1
        self.marker.pose.position.z = 0.1
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = np.sin(psiR/2)
        self.marker.pose.orientation.w = np.cos(psiR/2)


if __name__ == '__main__':
    #Initialize node
    rospy.init_node('vfhwp',anonymous=True)

    try:

        VFH()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
