#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 17:05:58 2019

@author: sadra
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from timeit import default_timer
from rg_rrt_star.symbolic_system.symbolic_system_rg_rrt_star import Oracle_RGRRTStar, PolytopeReachableSet
from pypolycontain.visualization.visualize_2D import visualize_2D_AH_polytope
from pypolycontain.lib.operations import distance_point_polytope
from rg_rrt_star.utils.visualization import visualize_node_tree_2D
import time
from datetime import datetime
import os
matplotlib.rcParams['font.family'] = "Times New Roman"

from rg_rrt_star.symbolic_system.examples.test_box_setup import oracle 

input_samples = 9
nonlinear_dynamic_step_size = 1e-2

global best_distance
#def test_box_planning():
global best_distance
best_distance=np.inf
initial_state = np.array([0,1,0,-1,1,1,1,0,0,0])
l = 0.1
p = 0.1
step_size = 4e-2
goal_state = np.array([-2,1,np.pi/4,-1,1,1,1,0,0,0])
goal_tolerance = 5e-2


def sampler():
    #TODO: return sample in state space
    return (np.random.rand(10)-0.5)*10



def contains_goal_function(reachable_set, goal_state):
    # TODO: determine whether goal is reached
    # if distance<0.5:
    #     # enumerate inputs
    #     potential_inputs = np.linspace(hopper_system.input_limits[0, 0], hopper_system.input_limits[1, 0],
    #                                    input_samples)
    #     for u_i in potential_inputs:
    #         state_list = []
    #         state = reachable_set.parent_state
    #         for step in range(int(step_size / nonlinear_dynamic_step_size)):
    #             state = hopper_system.forward_step(u=np.atleast_1d(u_i), linearlize=False, modify_system=False,
    #                                                  step_size=nonlinear_dynamic_step_size, return_as_env=False,
    #                                                  starting_state=state)
    #             state_list.append(state)
    #             distance = np.linalg.norm(goal_state-state)
#    best_distance = np.inf
#    for p in reachable_set.polytope_list:
#        distance = distance_point_polytope(p, goal_state)
#        best_distance = min(best_distance, distance)
#    if best_distance < goal_tolerance:
##        print('Goal error is %0.02f' % distance)
#        print('Goal reached?')
#        return True, [reachable_set.parent_state, goal_state]
    for p in reachable_set.polytope_list:
        if p.t[1]>2:
            return True, [reachable_set.parent_state, goal_state]                
    return False, None

def compute_reachable_set(state):
    print state
    list_of_reachable_sets = oracle(state) #TODO

    return PolytopeReachableSet(state,list_of_reachable_sets, contains_goal_function=contains_goal_function, \
                        reachable_set_step_size=step_size, use_true_reachable_set=False,\
                        nonlinear_dynamic_step_size=nonlinear_dynamic_step_size)

rrt = Oracle_RGRRTStar(initial_state, sampler, step_size, compute_reachable_set=compute_reachable_set, contains_goal_function=contains_goal_function,use_convex_hull=True,\
                               use_true_reachable_set=False)
found_goal = False
experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

duration = 0
os.makedirs('R3T_box_'+experiment_name)
max_iterations = 100
number_of_failures=0
start_time_all = time.time()
while rrt.node_tally<=200 or number_of_failures<=500:
    try:
        for itr in range(max_iterations):
            start_time = time.time()
            if rrt.build_tree_to_goal_state(goal_state, stop_on_first_reach=True, allocated_time= 5, rewire=False, explore_deterministic_next_state=False,save_true_dynamics_path =False) is not None:
                found_goal = True
            end_time = time.time()
            #get rrt polytopes
            polytope_reachable_sets = rrt.reachable_set_tree.id_to_reachable_sets.values()
            reachable_polytopes = []
            explored_states = []
            for prs in polytope_reachable_sets:
                reachable_polytopes.extend(prs.polytope_list)
                explored_states.append(prs.parent_state)
#                print "the size of explored states",len(explored_states)
            print "THE size of explored states",len(explored_states)
            # print(explored_states)
            # print(len(explored_states))
            # print('number of nodes',rrt.node_tally)
        #        print("Best distance %f" %best_distance)
            print "The number of nodes",rrt.node_tally
            if rrt.node_tally>=600:
                break
    except:
        print "Failure, ",number_of_failures
        number_of_failures+=1
    if found_goal:
        break

total_time=time.time()-start_time_all

from collections import deque
node_queue = deque([rrt.root_node])
all_states = []
i = 0
while node_queue:
    i+=1
    node = node_queue.popleft()
    all_states.append(np.ndarray.flatten(node.state))
    if node.children is not None:
        node_queue.extend(list(node.children))

"""
Visualization
"""
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt

def animate(X,ax1,fig,alpha):
    x,y,theta,x_1,y_1,x_2,y_2=X[0:7]
    a=1
    b=1
    R=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    # Good now plot
    ax1.set_xlabel("x",fontsize=30)
    ax1.set_ylabel("y",fontsize=30)
    ax1.set_xlim([-3.5*a,2*a])
    ax1.set_ylim([-0.02,3.5])
    fig.gca().set_aspect('equal')
    # Box
    up_right=np.array([x,y])+np.dot(R,[b,a])
    down_right=np.array([x,y])+np.dot(R,[b,-a])
    down_left=np.array([x,y])+np.dot(R,[-b,-a])
    up_left=np.array([x,y])+np.dot(R,[-b,a])
    bar=[patches.Polygon(np.array([[up_right[0],down_right[0],down_left[0],up_left[0]],[up_right[1],down_right[1],down_left[1],up_left[1]]]).reshape(2,4).T, True)]
    ax1.add_collection(PatchCollection(bar,color=(0.8,0.3,0.4),alpha=alpha,edgecolor=(0,0,0)))
    # Finger left
    L,h=0.1,0.3
    base_up,base_down,corner=np.array([x_1,y_1])+np.dot(R,[-h,-L]),np.array([x_1,y_1])+np.dot(R,[-h,L]),np.array([x_1,y_1])
    finger_left=[patches.Polygon(np.array([[base_up[0],base_down[0],corner[0]],[base_up[1],base_down[1],corner[1]]]).reshape(2,3).T, True)]
#    ax1.add_collection(PatchCollection(finger_left,color=(0.2,0.2,0.8),alpha=alpha,edgecolor=(0,0,0)))
    # Finger right
    L,h=0.1,0.3
    base_up,base_down,corner=np.array([x_2,y_2])+np.dot(R,[h,-L]),np.array([x_2,y_2])+np.dot(R,[h,L]),np.array([x_2,y_2])
    finger_right=[patches.Polygon(np.array([[base_up[0],base_down[0],corner[0]],[base_up[1],base_down[1],corner[1]]]).reshape(2,3).T, True)]
    ax1.add_collection(PatchCollection(finger_right,color=(0.2,0.2,0.8),alpha=alpha,edgecolor=(0,0,0)))    
    # Star 
#    ax1.plot([x_1,x_2],[y_1,y_2],'.',linewidth=3,markersize=10,color='blue')
#    ax1.plot([up_right[0]],[up_right[1]],'.',linewidth=3,markersize=10,color='orange')
    ax1.plot([-4,4],[0,0],linewidth=3,color=(0.1,0.1,0.1))
    ax1.grid(color=(0,0,0), linestyle='--', linewidth=0.5)
    # The contact forces
#    ax1.quiver([x-b*np.cos(theta)+a*np.sin(theta),x-b*np.cos(theta)+a*np.sin(theta)],[y-a*np.cos(theta)-b*np.sin(theta),y-a*np.cos(theta)-b*np.sin(theta)+U[0]],scale=10)

  
for H in [0,10,20,50,100,150,200,300,400,500,600,700,800,900,1000,1100,1172]:    
    fig,ax = plt.subplots()
    fig.set_size_inches(20, 16)
    ax.set_title(r"Reachable Box Poses after Explored %d Nodes"%H,fontsize=45)
#    for i in range(rrt.node_tally):
    for i in range(H):
        animate(all_states[i],ax,fig,alpha=0.7) 
    fig.savefig('R3T_box_'+experiment_name+'/reachable_box_%d.png'%i, dpi=100)


raise 1    
"""Back track and find a trajectory"""
largest_theta=0
for state in all_states:
    if state[2]>largest_theta:
        print "found a state"
        largest_theta=state[2]
        mystate=state
        
mynode=rrt.state_to_node_map[hash(str(mystate))]
traj=[]
parent_state=mystate
parent_node=mynode
while True:
    try:
        parent_node=parent_node.parent
        traj.append(parent_node.state)
    except:
        break

traj.reverse()
fig,ax = plt.subplots()
fig.set_size_inches(10, 8)
i=0
for x in traj:
    fig,ax = plt.subplots()
    fig.set_size_inches(10, 8)
    animate(x,ax,fig,alpha=0.8)
    fig.savefig('R3T_box_'+experiment_name+'/box_%d.png'%i, dpi=100)
    i+=1


fig,ax = plt.subplots()
fig.set_size_inches(20, 16)  
x=traj[0]
animate(x,ax,fig,alpha=0.8)
for x in traj:
    animate(x,ax,fig,alpha=0.8)
    fig.savefig('R3T_box_'+experiment_name+'/box_%d.png'%i, dpi=100)
    i+=1