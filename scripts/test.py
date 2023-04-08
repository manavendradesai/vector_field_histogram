from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
import numpy as np
import math

# ds = 0.05
# theta = np.pi/4
# lcms = 1.2
# npts = np.linspace(3,13,6,int)

# #Select npts points along this heading
# #Get relative xy positions of each cell in world frame
# xpts = ds*npts*math.cos(0)
# ypts = ds*npts*math.sin(0)

# #Get indices in 1-D costmap array
# #Column shift
# cs = np.rint(npts*math.cos(theta))
# #Row shift
# rs = np.rint(npts*math.sin(theta))
# #Index in the 1D costmap array
# I = np.rint(((lcms/ds)**2)*0.5) + (cs + (lcms/ds)*rs)

# print(np.rint(((lcms/ds)**2)*0.5))
# print(np.rint(lcms/ds))
# print(I)

# print(npts)
# print(npts*0.707)
# print(cs)
# print(rs)

# dcp = np.array([[1,1],[1,1]])
# Jcp = 10*np.exp(-dcp) 
# print(Jcp)

# temp = (2**2)**(0.5)
# print(temp)

# Jcp = np.array([[13,14,15,16],[9,10,11,12],[5,6,7,8],[1,2,3,4]])
# a = 2
# b = 3
# print(Jcp[a][b])
# fJcp = np.flip(Jcp,axis=0)
# #Reshape into the 1D array
# ODcp = np.reshape(fJcp,16)
# print(ODcp[(4-a-1)*4 + b + 1 - 1])

class Dog:

    #Class variables
    atr1 = "mammal"
    atr2 = "dog"

    def __init__(self, age):

        #Instance variable
        self.age = age

    def fun(self):
        print("I am a ", self.atr1)
        print("I am a ", self.atr2)


Rodger = Dog(10)

print(Rodger.atr1)
Rodger.fun()
print(Rodger.age)