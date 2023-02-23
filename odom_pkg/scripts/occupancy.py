import rospy
import array

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class Occupancy():
    def __init__(self):
        self.ogrid_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, occupancygrid_CB) # Subcribe to the occupancy grid
        self.ogrid_pub = rospy.Publisher('/occupancy2', String, queue_size=10)
        self.local_costmap_sub

    def occupancygrid_Pub():
	"""
	self.ogrid_pub = rospy.Publisher('/occupancy2', String, queue_size=10)
	"""
	rate = rospy.Rate(1) #define rate/sec 
	    
	while not rospy.is_shutdown():
	    rospy.loginfo()
	    self.ogrid_pub.publish()
	    rate.sleep()
		
    def occupancygrid_CB(self, msg) 
	    self.ogrid_1D = msg.data
	    
	    
    def occupancy_Sub():
	"""
	rospy.init_node('occupancy2', anonymous=True) #make node for occupancy grid
	self.ogrid_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, occupancygrid_CB) # Subcribe to the occupancy grid
	"""
	
	
    if __name__ == "__main__":
        rospy.init_node('occupancy', anonymous=True) #make node
        try:
            Occupancy()
            rospy.spin()
	except rospy.ROSInteruptException:
            pass
		
	
        
        
