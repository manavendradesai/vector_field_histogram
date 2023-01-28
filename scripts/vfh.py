from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
from matplotlib.patches import Arrow
import numpy as np
import math

#Metric system followed

#Initialize map details
mapX = 10                   #length
mapY = 10                   #breadth
ds = 0.05                   #grid cell size

#Initialize robot details
dia = 0.2                   #diameter
xR0 = np.array([2.8,3.0])
psi0 = np.pi/4              #X location, Y location, orientation   
xyR = xR0   
psiR = psi0              

#Goal pose
xG = np.array([0,3])
psiG = np.pi/2

#Initialize obstacle locations. 
#Lower left vertex
OA = np.array([1.5,2])             #obstacle A
Os = 1                             #obstacle size

#Obstacle centers
OAc = np.array([2,2.5])

#Local costmap dimension (a square)
lcms = 1.2                  #length

#Number of grid cells along one edge
ncm  = int(np.rint((lcms/ds)))

#Lethal distance and cost
ld = (Os/2)*1.414 + dia/2
LJ = 255
lJ = LJ*np.exp(-ld)

#Discretize heading angles
ndpsi = 36
vfhpsi = np.linspace(0,2*np.pi,ndpsi)

#Lookahead distance in terms of grid cells
LA = 11

#Plot initial environment
fig,ax=plt.subplots(1,1)

#Plot obstacles
obsA = Rectangle(OA,Os,Os,color='red',fill='true') 
ax.add_patch(obsA)

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

##############################################################

#Create a dummy costmap using the current robot location
#Provide positions
nx,ny = (ncm,ncm)
x = np.linspace(-1,1,nx)
y = np.linspace(1,-1,ny)
xcp,ycp = np.meshgrid(x,y)

dxcp = xcp*lcms/2 + xR0[0]
dycp = ycp*lcms/2 + xR0[1]

#Calculate distance based costs
dcp = ((dxcp-OAc[0])**2 + (dycp-OAc[1])**2)**(0.5) 

Jcp = LJ*np.exp(-dcp) 

#Plot initial costs
fig2,ax2=plt.subplots(1,1)
cp = ax2.contourf(xcp, ycp, Jcp)
fig2.colorbar(cp)
ax2.set_title('Euclidean distance based cost')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
plt.xlim(-lcms/2,lcms/2)
plt.ylim(-lcms/2,lcms/2)
plt.show()

#Create a 1D cost array
#Flip the cost matrix 
fJcp = np.flip(Jcp,axis=0)
#Reshape into the 1D array
ODcp = np.reshape(fJcp,ncm**2)

#Set previous VFH waypoint
xyvfh0 = xyR
xyvfh = xyR
psivfh = psiR
#VFH weights
wg = 5
wd = 1
wo = 1

CWX = 0
CWY = 0

PTSJ = 0

SI = 0

CS = 0
RS = 0

CSG = 0
RSG = 0

# while np.linalg.norm(xyR-xG)>dia:
for i in range(0,3):
#Select candidate waypoints on active circle boundary
#Select a heading

    #Set initial min VFH cost
    Jvfh = 10000

    #Discretize forward motion
    #Distance to goal in terms of grid cells
    d2G = np.int(np.linalg.norm(xyR-xG)/ds)
    #Check if goal is near
    n = min(LA,d2G)
    print(n)
    npts = np.linspace(3,n,5,int)
    print(npts)

    ctheta = 0

    for theta in vfhpsi:
        #Get relative xy positions of each cell in world frame
        #in grid cell count
        csg = npts*np.cos(theta)
        rsg = npts*np.sin(theta)

        #Get indices in 1-D costmap array
        #Column shift
        cs = csg
        # print(cs)
        #Row shift
        rs = rsg
        # print(rs)
        #Indices
        I = int((ncm**2)*0.5) + (cs + ncm*rs)
        #Retrieve costs at these indices
        cJ = ODcp[np.int_(I)]

        #Check if all costs are below critical cost lJ
        if np.all(cJ<lJ):
            
            ctheta = ctheta+1

            #Jump to furthest grid cell along direction theta
            #Candidate waypoint xy coordinates in world frame
            cwx = xyR[0] + ds*npts[-1]*np.cos(theta)
            cwy = xyR[1] + ds*npts[-1]*np.sin(theta)
            
            #Robot heading to goal
            thetag = np.arctan2((xG[1]-xyR[1]),(xG[0]-xyR[0]+0.001))
            #Robot heading to candidate waypoint
            thetawp = np.arctan2((cwy-xyR[1]),(cwx-xyR[0]+0.001)) 
            #Robot heading to previous waypoint
            thetawp0 = np.arctan2((xyvfh0[1]-xyR[1]),(xyvfh0[0]-xyR[0]+0.001))
            #Calculate VFH cost at the candidate waypoint
            J = (wg*(np.abs(thetag-thetawp)) + wd*(np.abs(thetawp-thetawp0)) + 
            wo*np.abs((psiR-thetawp)))

            #Compare candidate waypoint cost and current minimum
            if J<Jvfh:
                #Update candidate waypoint and min VFH cost
                xyvfh = np.array([cwx,cwy])
                psivfh = thetawp
                Jvfh = J
                CWX = xyR[0] + ds*npts[-1]*np.cos(theta)
                CWY = xyR[1] + ds*npts[-1]*np.sin(theta)
                PTSJ = cJ
                SI = I
                CS = cs
                RS = rs
                CSG = csg
                RSG = rsg

    print(CSG)
    print(RSG)
    print(CS)
    print(RS)
    print(SI)
    # print(ctheta)
    print(psivfh)
    print(CWX,CWY)
    print(PTSJ)
    arr = Arrow(xyR[0],xyR[1],-xyR[0]+CWX,-xyR[1]+CWY)

    #Update robot pose
    xyR = xyvfh
    psiR = psivfh
    xyvfh0 = xyvfh

    ########################################################

    #Update costmap
    dxcp = xcp*lcms/2 + xyR[0]
    # print(xcp)
    dycp = ycp*lcms/2 + xyR[1]
    # print(ycp)

    #Calculate distance based costs
    dcp = ((dxcp-OAc[0])**2 + (dycp-OAc[1])**2)**(0.5) 

    Jcp = LJ*np.exp(-dcp)

    # print(Jcp[ncm//2][ncm//2]) 

    #Create a 1D cost array
    #Flip the cost matrix 
    fJcp = np.flip(Jcp,axis=0)
    #Reshape into the 1D array
    ODcp = np.reshape(fJcp,ncm**2)

    # ########################################################

    #Plot updated environment
    fig = plt.figure()
    ax = fig.add_subplot(111)

    #Plot obstacles
    obsA = Rectangle(OA,Os,Os,color='red',fill='true') 
    ax.add_patch(obsA)

    # #Plot robot
    # rbt = Circle(xyR,radius=dia/2,color='blue',fill='true')
    # ax.add_patch(rbt)

    #Plot direction of motion
    ax.add_patch(arr)

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

    #Plot updated costmap
    fig2,ax2=plt.subplots(1,1)
    cp = ax2.contourf(xcp, ycp, Jcp)
    fig2.colorbar(cp)
    ax2.set_title('Euclidean distance based cost')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    plt.xlim(-lcms/2,lcms/2)
    plt.ylim(-lcms/2,lcms/2)
    plt.show()




