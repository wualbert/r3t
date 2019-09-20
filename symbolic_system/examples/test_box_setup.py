#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 14 14:10:56 2019

@author: sadra
"""

import numpy as np
from scipy.linalg import block_diag as blk

import pydrake.symbolic as sym

from pypolycontain.lib.zonotope import zonotope

from PWA_lib.Manipulation.contact_point_pydrake import contact_point_symbolic_2D
from PWA_lib.Manipulation.system_symbolic_pydrake import system_symbolic
from PWA_lib.Manipulation.system_numeric import system_hard_contact_PWA_numeric as system_numeric
from PWA_lib.Manipulation.system_numeric import environment,merge_timed_vectors_glue,merge_timed_vectors_skip,\
    trajectory_to_list_of_linear_cells,trajectory_to_list_of_linear_cells_full_linearization,\
    PWA_cells_from_state,hybrid_reachable_sets_from_state,environment_from_state
from PWA_lib.trajectory.poly_trajectory import point_trajectory_tishcom,\
    polytopic_trajectory_given_modes,point_trajectory_given_modes_and_central_traj,\
    point_trajectory_tishcom_time_varying


mysystem=system_symbolic("Picking Up a Box", 2)
x,y,theta=sym.Variable("x"),sym.Variable("y"),sym.Variable("theta")
x_1,y_1,x_2,y_2=sym.Variable("x_1"),sym.Variable("y_1"),sym.Variable("x_2"),sym.Variable("y_2")
mysystem.q_o=np.array([x,y,theta])
mysystem.q_m=np.array([x_1,y_1,x_2,y_2])

a=1 # The half of the length of the vertical side
b=1 # The half the length of the horizonatl side
# Dynamics:
mysystem.C=np.zeros((3,3))
g=9.8
mysystem.tau_g=np.array([0,-g,0])
mysystem.B=np.zeros((3,0))

# Mass Matrix
M=1
I=M/3.0*a**2
mysystem.M=blk(*[M,M,I])
mysystem.M_inv=np.linalg.inv(mysystem.M)

# Contact Point 1: The ground with left corner
phi_1= y - a * sym.cos(theta) - b * sym.sin(theta)
psi_1= - (x + a * sym.sin(theta) - b * sym.cos(theta) )
J_1n=np.array([0,1,-b*sym.cos(theta)-a*sym.sin(theta)]).reshape(3,1)
J_1t=np.array([1,0,a*sym.cos(theta)+b*sym.sin(theta)]).reshape(3,1)
J_1=np.hstack((J_1n,J_1t))
C1=contact_point_symbolic_2D(mysystem,phi=phi_1,psi=psi_1,J=J_1,friction=0.9,name="contact point")
#C1.sliding=False
#C1.sticking=False
C1.no_contact=False


# Contact Point 2: The ground with right corner
phi_2= y - a * sym.cos(theta) + b * sym.sin(theta)
psi_2= - ( x + a * sym.sin(theta) + b * sym.cos(theta) )
J_2n=np.array([0,1,b*sym.cos(theta)-a*sym.sin(theta)]).reshape(3,1)
J_2t=np.array([1,0,a*sym.cos(theta)-b*sym.sin(theta)]).reshape(3,1)
J_2=np.hstack((J_2n,J_2t))
C2=contact_point_symbolic_2D(mysystem,phi=phi_2,psi=psi_2,J=J_2,friction=0.8,name="contact point")
#C2.sliding=False
#C2.sticking=False
#C2.no_contact=False


# Contact Point 3: Left finger
phi_3 = (x-x_1)*sym.cos(theta) - (y_1-y)*sym.sin(theta)- b 
psi_3 = (y_1-y)*sym.cos(theta) + (x-x_1)*sym.sin(theta)
J_3n=np.array([sym.cos(theta),sym.sin(theta),psi_3]).reshape(3,1)
J_3t=np.array([-sym.sin(theta),sym.cos(theta),-b]).reshape(3,1)
J_3=np.hstack((J_3n,J_3t))
C3=contact_point_symbolic_2D(mysystem,phi=phi_3,psi=psi_3,J=J_3,friction=0.5,name="contact point")
C3.sliding=False
C3.sticking=False
C3.no_contact=True


# Contact Point 4: Right finger
phi_4 = (x_2-x)*sym.cos(theta) + (y_2-y)*sym.sin(theta)- b 
psi_4 = (y_2-y)*sym.cos(theta) - (x_2-x)*sym.sin(theta)
J_4n=np.array([-sym.cos(theta),-sym.sin(theta),psi_4]).reshape(3,1)
J_4t=np.array([-sym.sin(theta),sym.cos(theta),b]).reshape(3,1)
J_4=np.hstack((J_4n,J_4t))
C4=contact_point_symbolic_2D(mysystem,phi=phi_4,psi=psi_4,J=J_4,friction=0.8,name="contact point")
#C4.sliding=False
#C4.sticking=False
C4.no_contact=True


# Build the Symbolic MLD system
mysystem.build_and_linearize()
sys=system_numeric(mysystem)

if True:
    # Add a numerical environment
    Eta=environment(0)
    Eta.dict_of_values={x:0,y:a,theta:0,x_1:-b,x_2:b,y_1:a,y_2:a,
         mysystem.v_o[0]:0,mysystem.v_o[1]:0,mysystem.v_o[2]:0,
         mysystem.u_lambda[0]:0,mysystem.u_lambda[1]:0,mysystem.u_lambda[2]:0,mysystem.u_lambda[3]:0,
         mysystem.u_lambda[4]:0,mysystem.u_lambda[5]:0,mysystem.u_lambda[6]:0,mysystem.u_lambda[7]:0,
         mysystem.u_m[0]:0,mysystem.u_m[1]:0,mysystem.u_m[2]:0,mysystem.u_m[3]:0,
         mysystem.h:0.1}
    epsilon_max=np.array([20,20,np.pi/2,10,10,10,10,50,50,50,1,1,1,1,500,500,500,500,70,70,70,70]).reshape(22,1)
    epsilon_min=-np.array([20,20,np.pi/2,10,0,10,0,50,50,50,1,1,1,1,500,500,500,500,70,70,70,70]).reshape(22,1)
    sys.add_environment(Eta,epsilon_max,epsilon_min)

x_0=np.array([0,a,0,-b,a,b,a,0,0,0]).reshape(10,1)
#list_of_reachable_sets=hybrid_reachable_sets_from_state(mysystem,x_0,h,epsilon_min,epsilon_max)

def oracle(state,h=0.1):
    return hybrid_reachable_sets_from_state(mysystem,state.reshape(10,1),h,epsilon_min,epsilon_max)