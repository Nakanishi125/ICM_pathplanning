#!/usr/bin/env python
# coding: utf-8

# In[2]:


import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time
import math


# In[60]:


Data = pd.read_csv("../visualize.csv", header=None)

lx = Data.iloc[:20,0]
ly = Data.iloc[:20,1]
lx = lx.values.tolist()
ly = ly.values.tolist()
lxlist = [lx[5*(i-1):5*i] for i in range(5)]
lxlist.pop(0)
lylist = [ly[5*(i-1):5*i] for i in range(5)]
lylist.pop(0)


rx = Data.iloc[20:,0]
ry = Data.iloc[20:,1]
rx = rx.values.tolist()
ry = ry.values.tolist()
rxlist = [rx[5*(i-1):5*i] for i in range(5)]
rxlist.pop(0)
rylist = [ry[5*(i-1):5*i] for i in range(5)]
rylist.pop(0)

sxlist = rx[20:29]
sylist = ry[20:29]


# In[59]:

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.set(xlim=(-200,400),ylim=(-10,600))
ax.set_aspect('equal')

for i in range(len(lxlist)):
	ax.plot(lxlist[i], lylist[i])
for i in range(len(rxlist)):
	ax.plot(rxlist[i], rylist[i])
ax.plot(sxlist, sylist)

plt.savefig("../positioning.svg")


	
