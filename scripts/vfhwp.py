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
        #VFH critical cost
        self.crit_cost = rospy.get_param('/vfh/crit_cost',75)
        #Number of grid cells along one edge
        self.ncm = rospy.get_param('/vfh/ncm',20)

        #Angle discretization
        self.ndpsi = rospy.get_param('/vfh/ndpsi',36)

        # #Get goal
        # self.xG = np.array([1.8,0])
        # self.psiG = np.pi/2

        #Get local costmap resolution
        self.ds = rospy.get_param('/move_base/local_costmap/resolution',0.05)

        # #Get initial position and pose
        # self.xR0 = np.array([-0.6,0])

        #Define subscriber to odom
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.robot_pose)

        #Define subscriber to local_costmap data
        self.local_costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap",
        OccupancyGrid,self.local_costmap)

        #Define publisher for marker
        self.marker_pub = rospy.Publisher("/visualization_marker",Marker,queue_size=2)


    #Receive robot pose
    def robot_pose(self,pose_msg):
        self.X = pose_msg.pose.pose.position.x
        self.Y = pose_msg.pose.pose.position.y
        psiz = pose_msg.pose.pose.orientation.z
        psiw = pose_msg.pose.pose.orientation.w
        self.psi = 2*np.arctan((psiz)/(0.001+psiw))

    #Recieve local costmap data
    #local_costmap is postioned in /odom frame
    def local_costmap(self,costmap_msg):
        self.local_cost = costmap_msg.data

    #Determine VFH waypoint
    def waypoint(self):

        #Collect costmap
        ODcp = self.local_cost
        #Collect critical cost
        crit_cost = self.crit_cost

        #Collect previous waypoint position and pose
        xyvfh0 = self.xR0

        #Collect robot position and pose
        xyR = np.array([self.X,self.Y])
        psiR = self.psi

        #Collect goal position and pose
        xG = self.xG
        psiG = self.psiG

        #Collect grid cell size
        ds = self.ds
        #Number of grid cells along one edge
        ncm = self.ncm
        #Collect lookahead distance
        LA = self.LA

        #VFH weights
        wg = self.wg
        wd = self.wd
        wo = self.wo

        #Discretize heading angles
        ndpsi = 36
        vfhpsi = np.linspace(0,2*np.pi,ndpsi)

        #Initialize candidate waypoint location and pose
        xyvfh = 0
        psivfh = 0

        #Initialize VFH cost
        Jvfh = 10000

        #Discretize forward motion
        #Distance to goal in terms of grid cells
        d2G = np.int(np.linalg.norm(xyR-xG)/ds)
        #Check if goal is nearer
        n = min(LA,d2G)
        # print(n)
        npts = np.linspace(3,n,5,int)
        # print(npts)

        for theta in vfhpsi:
            #Get indices in 1-D costmap array
            #Get relative xy positions of each cell in world frame
            #in grid cell count
            
            #Column shift
            cs = npts*np.cos(theta)
            #Row shift
            rs = npts*np.sin(theta)
            #Indices
            csg = ncm//2 + 1 + cs - 1
            rsg = ncm//2 + 1 - rs - 1
            I = (ncm-rsg-1)*ncm + csg + 1 - 1

            #Retrieve costs at these indices
            cJ = ODcp[np.int_(I)]

            #Check if all costs are below critical cost lJ
            if np.all(cJ<crit_cost):

                #Jump to furthest grid cell along direction theta
                #Candidate waypoint xy coordinates in world frame
                cwx = xyR[0] + ds*npts[-1]*np.cos(theta)
                cwy = xyR[1] + ds*npts[-1]*np.sin(theta)
                
                #Robot heading to goal
                thetag = np.mod(2*np.pi + np.arctan2((xG[1]-xyR[1]),(xG[0]-xyR[0]+0.001)), 2*np.pi)
                #Robot heading to candidate waypoint
                # thetawp = np.mod(2*np.pi + np.arctan2((cwy-xyR[1]),(cwx-xyR[0]+0.001)), 2*np.pi) 
                thetawp = theta
                #Robot heading to previous waypoint
                thetawp0 = np.mod(2*np.pi + np.arctan2((xyvfh0[1]-xyR[1]),(xyvfh0[0]-xyR[0]+0.001)), 2*np.pi)
                #Calculate VFH cost at the candidate waypoint
                J = (wg*(np.abs(thetag-thetawp)) + wd*(np.abs(thetawp-thetawp0)) + 
                wo*np.abs((psiR-thetawp)))

                #Compare candidate waypoint cost and current minimum
                if J<=Jvfh:
                    #Update candidate waypoint and min VFH cost
                    xyvfh = np.array([cwx,cwy])
                    psivfh = thetawp*(n==LA) + psiG*(n!=LA)
                    Jvfh = J

        self.pub_marker(xyvfh[0],xyvfh[1],psivfh)


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
        self.marker.color.g = 1
        self.marker.color.b = 0
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



if __name__ == '__main__':
    #Initialize node
    rospy.init_node('vfhwp',anonymous=True)

    try:

        VFH()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
