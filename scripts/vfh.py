from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
import numpy as np
import math

#Metric system followed

#Initialize map details
mapX = 10                   #length
mapY = 10                   #breadth
ds = 0.05                   #grid cell size

#Initialize robot details
dia = 0.2                   #diameter
xR0 = np.array([-2,-2])
psi0 = np.pi/4              #X location, Y location, orientation   
xyR = xR0                 

#Goal pose
xG = np.array([1,3])
psiG = np.pi/2

#Initialize obstacle locations. Lower left vertex
OA = np.array([-1.5,2.5])             #obstacle A
OB = np.array([2.5,1.5])              #obstacle B
OC = np.array([0.5,-1.5])             #obstacle C
Os = 1                                #obstacle size

#Obstacle centers
OAc = np.array([-1,3])
OBc = np.array([3,2])
OCc = np.array([1,-1])

#Local costmap dimension (a square)
lcms = 1.2                  #length

#Number of grid cells along one edge
ncm  = int(np.rint((lcms/ds)))

#Lethal distance and cost
ld = (Os/2)*1.414 + dia/2
LJ = 255
lJ = LJ*math.exp(-ld)

#Discretize heading angles
ndpsi = 35
vfhpsi = np.linspace(0,2*np.pi,ndpsi)

#Discretize forward motion
npts = np.linspace(3,13,6,int)

##############################################################

#Create a dummy costmap using the current robot location
#Provide positions
nx,ny = (ncm,ncm)
x = np.linspace(-1,1,nx)
y = np.linspace(1,-1,ny)
xcp,ycp = np.meshgrid(x,y)

xcp = xcp*lcms/2 + xR0[0]
ycp = ycp*lcms/2 + xR0[1]

#Calculate distance based costs
dcp = ((xcp-OAc[0])**2 + (ycp-OAc[1])**2)**(0.5)
Jcp = LJ*np.exp(-dcp) 

#Plot costs
fig2,ax2=plt.subplots(1,1)
cp = ax2.contourf(xcp, ycp, Jcp)
fig2.colorbar(cp)
ax2.set_title('Euclidean distance based cost')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
plt.show()

#Create a 1D cost array
#Flip the cost matrix 
fJcp = np.flip(Jcp,axis=0)
#Reshape into the 1D array
ODcp = np.reshape(fJcp,ncm**2)

#Set initial min VFH cost
Jvfh = 10000
#Set previous VFH waypoint
xyvfh0 = xG
xyvfh = xG
#VFH weights
wg = 3
wd = 1

while np.linalg.norm(xyR-xG)>dia:
#Select candidate waypoints on active circle boundary
#Select a heading
    for theta in vfhpsi:
        #Select npts points along this heading
        #Get relative xy positions of each cell in world frame
        #in grid cell count
        csg = np.rint(npts*math.cos(theta))
        rsg = np.rint(npts*math.sin(theta))

        #Get indices in 1-D costmap array
        #Column shift
        cs = np.int_(csg)
        print(cs)
        #Row shift
        rs = np.int_(rsg)
        print(rs)
        #Indices
        I = int((ncm**2)*0.5) + (cs + ncm*rs)
        print(I)
        #Retrieve costs at these indices
        cJ = ODcp[np.int_(I)]

        #Check if all costs are below critical cost lJ
        if np.all(cJ<lJ):
            
            #Candidate waypoint xy coordinates in world frame
            cwx = xR0[0] + ds*npts[-1]*math.cos(theta)
            cwy = xR0[1] + ds*npts[-1]*math.sin(theta)
            
            #Robot heading to goal
            thetag = np.arctan2((xG[1]-xR0[1]),(xG[0]-xR0[0]+0.001))
            #Robot heading to candidate waypoint
            thetawp = np.arctan2((cwy-xR0[1]),(cwx-xR0[0]+0.001)) 
            #Robot heading to previous waypoint
            thetawp0 = np.arctan2((xyvfh0[1]-xR0[1]),(xyvfh0[0]-xR0[0]+0.001))
            #Calculate VFH cost at the candidate waypoint
            J = wg*(np.abs(thetag-thetawp)) + wd*(np.abs(thetawp-thetawp0))

            #Compare candidate waypoint cost and current minimum
            if J<Jvfh:
                #Update waypoint and min VFH cost
                xyvfh = (cwx,cwy)
                Jvfh = J
                xyvfh0 = xyvfh

    #Update robot position
    xyR = xyvfh

    ##############################################################

    #Plot environment
    fig = plt.figure()
    ax = fig.add_subplot(111)

    #Plot obstacles
    obsA = Rectangle(OA,Os,Os,color='red',fill='true') 
    obsB = Rectangle(OB,Os,Os,color='red',fill='true')
    obsC = Rectangle(OC,Os,Os,color='red',fill='true')
    ax.add_patch(obsA)
    ax.add_patch(obsB)
    ax.add_patch(obsC)

    #Plot robot
    rbt = Circle(xyR,radius=dia/2,color='blue',fill='true')
    ax.add_patch(rbt)

    #Plot goal
    goal = Circle(xG,radius=dia/2,color='green',fill='true')
    ax.add_patch(goal)

    plt.title('Environment')
    plt.xlim(-mapX/2,mapX/2)
    plt.ylim(-mapY/2,mapY/2)
    ax.axis('equal')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.show()

