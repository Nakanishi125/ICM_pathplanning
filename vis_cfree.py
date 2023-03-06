import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

if __name__ == '__main__':
    # Change the below file path
    data = pd.read_csv('../cluster.csv')

    x = data.iloc[:,0]
    y = data.iloc[:,1]
    z = data.iloc[:,2]

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(x, y, z)

    ax.set_xlabel('x', size=18)
    ax.set_xlim(-400, 400)
    ax.set_ylabel('y', size=18)
    ax.set_ylim(-50, 450)
    ax.set_zlabel('phi', size=18)
    ax.set_zlim(0, 360)
    ax.view_init(elev=20, azim=-45)
    #ax.view_init(elev=90, azim=-90)
    #ax.set_aspect('equal')

    fig.savefig('../cfree_pointcloud.svg')
 
###############################################################################
    fig_plane = plt.figure(facecolor='skyblue')
    ax_xy = fig_plane.add_subplot(1, 3, 1)
    ax_xphi = fig_plane.add_subplot(1, 3, 2)
    ax_yphi = fig_plane.add_subplot(1, 3, 3)

    ax_xy.set_xlabel('x')
    ax_xy.set_ylabel('y')
    ax_xy.set_xlim(-300, 300)
    ax_xy.set_ylim(-10, 450)

    ax_xphi.set_xlabel('x')
    ax_xphi.set_ylabel('phi')

    ax_yphi.set_xlabel('y')
    ax_yphi.set_ylabel('phi')

    ax_xy.scatter(x, y)
    ax_xphi.scatter(x, z)
    ax_yphi.scatter(y, z)

    plt.tight_layout()
    fig_plane.savefig('../cfree_planar.svg')
