#!/usr/bin/env python
# coding: utf-8

# In[1]:

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# In[6]:


def deg_to_rad(deg):
    rad = deg*3.141592/180
    return rad


# In[11]:


def generate_vertex(ref:list, w, h, th):
    x = []
    y = []
    x.append(ref[0])
    y.append(ref[1])
    x2 = ref[0]+w*np.cos(th)
    y2 = ref[1]+w*np.sin(th)
    x.append(x2)
    y.append(y2)
    x3 = x2 - h*np.sin(th)
    y3 = y2 + h*np.cos(th)
    x.append(x3)
    y.append(y3)
    x4 = x3 - w*np.cos(th)
    y4 = y3 - w*np.sin(th)
    x.append(x4)
    y.append(y4)
    x.append(ref[0])
    y.append(ref[1])

    return x,y

def generate_vertex2(ref:list, w, h, th):
    x = []
    y = []
    x.append(ref[0])
    y.append(ref[1])
    x2 = ref[0]-w*np.cos(th)
    y2 = ref[1]-w*np.sin(th)
    x.append(x2)
    y.append(y2)
    x3 = x2 + h*np.sin(th)
    y3 = y2 - h*np.cos(th)
    x.append(x3)
    y.append(y3)
    x4 = x3 + w*np.cos(th)
    y4 = y3 + w*np.sin(th)
    x.append(x4)
    y.append(y4)
    x.append(ref[0])
    y.append(ref[1])

    return x,y

# In[9]:


fn = input("file path from here:")
df = pd.read_csv(fn, header=None, skiprows=[0])
#df = pd.read_csv(fn, header=None)
print(df)


# In[29]:


height = [38, 130, 130, 90]
width = [60, 35, 35, 35]

fig = plt.figure()

for i in range(0, len(df)):
    
    ax = fig.add_subplot(1,1,1)
    ax.set(xlim=(-250,250),ylim=(-10,500))
    ax.set_aspect('equal')

    t1 = df.iloc[i][0]
    t2 = df.iloc[i][1]
    t3 = df.iloc[i][2]
    t4 = df.iloc[i][3]
    t5 = df.iloc[i][4]
    t6 = df.iloc[i][5]

    # Left
    AbsAngle = 0.0
    center = [-30 ,0]
    ref = [center[0]-0.5*width[0], center[1]]
    x,y = generate_vertex(ref, width[0], height[0], 0.0)
    center[0] = center[0] - height[0]*np.sin(0)
    center[1] = center[1] + height[0]*np.cos(0)
    ax.plot(x,y)

    AbsAngle = AbsAngle + t1
    rad = deg_to_rad(AbsAngle)
    ref = [center[0]-0.5*width[1]*np.cos(rad), center[1]-0.5*width[1]*np.sin(rad)]
    x,y = generate_vertex(ref, width[1], height[1], rad)
    center[0] = center[0] - height[1]*np.sin(rad)
    center[1] = center[1] + height[1]*np.cos(rad)
    ax.plot(x,y)

    AbsAngle = AbsAngle + t2
    rad = deg_to_rad(AbsAngle)
    ref = [center[0]-0.5*width[2]*np.cos(rad), center[1]-0.5*width[2]*np.sin(rad)]
    x,y = generate_vertex(ref, width[2], height[2], rad)
    center[0] = center[0] - height[2]*np.sin(rad)
    center[1] = center[1] + height[2]*np.cos(rad)
    ax.plot(x,y)

    AbsAngle = AbsAngle + t3
    rad = deg_to_rad(AbsAngle)
    ref = [center[0]-0.5*width[3]*np.cos(rad), center[1]-0.5*width[3]*np.sin(rad)]
    x,y = generate_vertex(ref, width[3], height[3], rad)
    center[0] = center[0] - height[3]*np.sin(rad)
    center[1] = center[1] + height[3]*np.cos(rad)
    ax.plot(x,y)    

    # Right hand
    AbsAngle = 0.0
    center = [30.01 ,415]
    ref = [center[0]+0.5*width[0], center[1]]
    x,y = generate_vertex2(ref, width[0], height[0], 0.0)
    center[0] = center[0] + height[0]*np.sin(0)
    center[1] = center[1] - height[0]*np.cos(0)
    ax.plot(x,y)

    AbsAngle = AbsAngle + t4
    rad = deg_to_rad(AbsAngle)
    ref = [center[0]+0.5*width[1]*np.cos(rad), center[1]+0.5*width[1]*np.sin(rad)]
    x,y = generate_vertex2(ref, width[1], height[1], rad)
    center[0] = center[0] + height[1]*np.sin(rad)
    center[1] = center[1] - height[1]*np.cos(rad)
    ax.plot(x,y)

    AbsAngle = AbsAngle + t5
    rad = deg_to_rad(AbsAngle)
    ref = [center[0]+0.5*width[2]*np.cos(rad), center[1]+0.5*width[2]*np.sin(rad)]
    x,y = generate_vertex2(ref, width[2], height[2], rad)
    center[0] = center[0] + height[2]*np.sin(rad)
    center[1] = center[1] - height[2]*np.cos(rad)
    ax.plot(x,y)

    AbsAngle = AbsAngle + t6
    rad = deg_to_rad(AbsAngle)
    ref = [center[0]+0.5*width[3]*np.cos(rad), center[1]+0.5*width[3]*np.sin(rad)]
    x,y = generate_vertex2(ref, width[3], height[3], rad)
    center[0] = center[0] + height[3]*np.sin(rad)
    center[1] = center[1] - height[3]*np.cos(rad)
    ax.plot(x,y)
    
    if i == len(df)-1 :
        plt.show()
#    elif i == 1:
#        plt.pause(10)
#        plt.clf()
    else:
        plt.pause(0.01)
        plt.clf()

plt.close()
