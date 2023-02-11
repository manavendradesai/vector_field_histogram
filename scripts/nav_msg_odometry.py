#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import String

odom1_pose = (float, float, float)

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

if __name__ == "__main__":
    rospy.init_node('odometry', anonymous=True) #make node 
    odom_sub = rospy.Subscriber('/odom',Odometry,odometryCb)
    try:
        publisher()
    except rospy.ROSInterruptException:
    	pass
    	
    rospy.spin()
    
print(odom1_pose)

