#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  6 13:45:03 2019

@author: sadra
"""

import numpy as np

"""
Visualization for Hopper
"""
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt


def hopper_plot(X,ax,fig):
    x,y,r,theta,phi=X[0:5]
    w_1=0.2
    w_2=0.2
    h=0.2
    L=5
    a=3
    R=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    # Good now plot
    ax.set_xlabel("x",fontsize=20)
    ax.set_ylabel("y",fontsize=20)
    ax.set_xlim([-5,10])
    ax.set_ylim([-5,10])
    fig.gca().set_aspect('equal')
    # leg
    corner=np.array([x,y])
    down_left=np.array([x,y])+np.dot(R,[-w_1,h])
    down_right=np.array([x,y])+np.dot(R,[w_1,h])
    up_left=np.array([x,y])+np.dot(R,[-w_1,h])+np.dot(R,[-w_1/2,L])
    up_right=np.array([x,y])+np.dot(R,[w_1,h])+np.dot(R,[w_1/2,L])
    leg=[patches.Polygon(np.array([[corner[0],down_right[0],up_right[0],up_left[0],down_left[0]],\
                                   [corner[1],down_right[1],up_right[1],up_left[1],down_left[1]]]).reshape(2,5).T, True)]
    ax.add_collection(PatchCollection(leg,color=(0.8,0.3,0.4),alpha=0.8,edgecolor=(0,0,0)))
    # Body
    center=np.array([x,y])+np.dot(R,[0,r])
    R=np.array([[np.cos(phi),-np.sin(phi)],[np.sin(phi),np.cos(phi)]])
    up_right,up_left,down_left,down_right=center+np.dot(R,[a,w_2]),\
                                            center+np.dot(R,[-a,w_2]),\
                                            center+np.dot(R,[-a,-w_2]),\
                                            center+np.dot(R,[a,-w_2])
    body=[patches.Polygon(np.array([[up_right[0],up_left[0],down_left[0],down_right[0]],\
                                    [up_right[1],up_left[1],down_left[1],down_right[1]]]).reshape(2,4).T, True)]
    ax.add_collection(PatchCollection(body,color=(0.2,0.2,0.8),alpha=0.8,edgecolor=(0,0,0)))
    ax.plot([-10,10],[0,0],'-',linewidth=3,markersize=10)
    ax.grid(color=(0,0,0), linestyle='--', linewidth=0.5)

fig,ax = plt.subplots()
fig.set_size_inches(10, 10)
X=[0,0.2,3.5,0.2,0.5]
hopper_plot(X,ax,fig)       
#def generate_figures():    
#    for t in range(T+1):
#        fig,ax = plt.subplots()
#        fig.set_size_inches(10, 10)
#        if t!=T:
#            ax.set_title(r'%d/%d %s'%(t,T,mysystem.name)+'\n'+
#                         r'$\lambda^{ground}_n: %0.1f * \lambda^{ground}_t: %0.1f $'
#                         %(u_lambda[t][0],u_lambda[t][1])
#                         +'\n'+r'$\lambda^{rF}_n: %0.1f * \lambda^{rF}_t: %0.1f $'
#                         %(u_lambda[t][2],u_lambda[t][3])
#                         +'\n'+r'$\lambda^{lF}_n: %0.1f * \lambda^{lF}_t: %0.1f $'
#                         %(u_lambda[t][4],u_lambda[t][5]))
#        animate(x_traj[t],ax,fig)
#        if t==-1:
#            ax.arrow(-1.8, 0.1, 0.3, 0.0, head_width=0.3, head_length=0.3, linewidth=10, fc='k', ec='k')
#        fig.savefig('figures/carrot_%d.png'%t, dpi=100)