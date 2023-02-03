#! /usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

def vfhwp():

    #Initialize node
    rospy.init_node('vfhwp',anonymous=True)

    #Get parameters
    #Robot diameter
    dia = rospy.get_param('/vfh/dia',0.2)
    #VFH weights
    wg = rospy.get_param('/vfh/wg',3)
    wo = rospy.get_param('/vfh/wo',1)
    wd = rospy.get_param('/vfh/wd',1)
    #Lookahead distance
    LA = rospy.get_param('/vfh/LA',11)

    #Subscribe to robot pose
    rospy.Subscriber("/amcl_pose",)

    #Marker publisher
    marker_pub = rospy.Publisher("/visualization_marker",Marker,queue_size=2)






    #Define marker
    marker = Marker()

    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    marker.type = 0
    marker.id = 0

    marker.scale.x = 0.3
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1

    psiR = np.pi

    marker.pose.position.x = 1
    marker.pose.position.y = 1
    marker.pose.position.z = 0.1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = np.sin(psiR/2)
    marker.pose.orientation.w = np.cos(psiR/2)

    rate = rospy.Rate(10)

    #Publish marker at waypoint
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rate.sleep()


if __name__ == '__main__':
    try:
        vfhwp()
    except rospy.ROSInterruptException:
        pass
