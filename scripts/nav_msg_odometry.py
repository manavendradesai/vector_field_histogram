#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
# include in manifest.xml file ? <depend package="nav_msgs"/> ?

class OdomData:
    
    def __init__(self):

        #Define subscriber to odom
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.robot_pose)

        #Define publisher to odom
        self.odom_pub = rospy.Publisher("/odom2", String, queue_size=10)

    #Callback function
    def robot_pose(self, pose_msg):

        #Robot position
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        self.z = pose_msg.pose.pose.orientation.z

        print(self.x, self.y, self.z)


"""

def odometryCb(msg):
    global odom1_pose
    odom1_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z)
    print(msg.pose.pose)
    
def publisher():
    pub = rospy.Publisher('/odom2',String, queue_size=10)
    
   #rospy.init_node('odometry2', anonymous=True) #make node
    
    rate = rospy.Rate(1) #define rate/sec 
    
    while not rospy.is_shutdown():
        data = "This is odom 2"
        
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
"""

if __name__ == "__main__":
    rospy.init_node('nav_msg_odometry', anonymous=True) #make node 

    try:
        OdomData()
        rospy.spin()
    except rospy.ROSInterruptException:
    	pass
    	
    
