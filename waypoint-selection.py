import numpy as np
import matplotlib.pyplot as plt

def cellCost(x-pos, y-pos):
    return costVal 

##def waypointSelector(curDegree, nxtDegree):
##    xLowCost = 0
##    yLowCost = 0
##    critThreshold = 80
##    waypoint = (xLowCost, yLowCost)
##    return waypoint

critThreshold = 80
waypoint(0,0)

for i in range(0, 359, 10):
    print("Degree of coordinate: ", i)
    for j in range(3, 14, 2):
        if (j == 3):
            print("First Point: ",j)
            xPoint1=j * cos(i)
            yPoint1=j * sin(i)
            if (cellCost(xPoint1, yPoint1)> critThreshold)
                break
        if (j == 5):
            print("Second Point: ",j)
            xPoint2=j * cos(i)
            yPoint2=j * sin(i)
            if (cellCost(xPoint2, yPoint2)> critThreshold)
                break
        if (j == 7):
            print("Third Point: ",j)
            xPoint3=j * cos(i)
            yPoint3=j * sin(i)
            if (cellCost(xPoint3, yPoint3)> critThreshold)
                break
        if (j == 9):
            print("Fourth Point: ",j)
            xPoint4=j * cos(i)
            yPoint4=j * sin(i)
            if (cellCost(xPoint4, yPoint4)> critThreshold)
                break
        if (j == 11):
            print("Fifth Point: ",j)
            xPoint5=j * cos(i)
            yPoint5=j * sin(i)
            if (cellCost(xPoint5, yPoint5)> critThreshold)
                break
        if (j == 13):
            print("Sixth Point: ",j)
            xPoint6=j * cos(i)
            yPoint6=j * sin(i)
            if (cellCost(xPoint6, yPoint6)> critThreshold)
                break
            else
                if (cellCost(xPoint6, yPoint6) < cellCost(waypoint))
                    waypoint = (xPoint6, yPoint6)
        # Check first cell up to last cell in 1D cost map array
        # to see if all costs are less than critical cost
        

        # if statement check each point in 1D is less than crit cost

            #Nested if statement check if last point is a candidate
