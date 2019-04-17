import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import pickle
import os
from dubin_car.base_reachable_set import *

def visualize_3d_scatter(base_reachable_set,fig = None):
    reachable_points = np.argwhere(base_reachable_set.is_reachables)
    xs = reachable_points[:,0]*base_reachable_set.x_resolution+base_reachable_set.x_range[0]
    ys = reachable_points[:,1]*base_reachable_set.y_resolution+base_reachable_set.y_range[0]
    thetas = reachable_points[:,2]*base_reachable_set.theta_resolution-np.pi
    if fig is None:
        fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xs,ys,thetas, s=1, alpha=0.5, marker='.')
    ax.set_xlim(base_reachable_set.x_range[0]*1.1,base_reachable_set.x_range[1]*1.1)
    ax.set_ylim(base_reachable_set.y_range[0]*1.1,base_reachable_set.y_range[1]*1.1)
    ax.set_zlim(-np.pi, np.pi)
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_zlabel('$theta$(rad)')
    ax.set_title('Base Reachable Set with Turn Radius %f m' % base_reachable_set.turn_radius)
    return fig, ax

def visualize_2d_quiver(base_reachable_set, fig=None):
    reachable_points = np.argwhere(base_reachable_set.is_reachables)
    xs = reachable_points[:,0]*base_reachable_set.x_resolution+base_reachable_set.x_range[0]
    ys = reachable_points[:,1]*base_reachable_set.y_resolution+base_reachable_set.y_range[0]
    thetas = reachable_points[:,2]*base_reachable_set.theta_resolution-np.pi
    us = np.cos(thetas)
    vs = np.sin(thetas)
    if fig is None:
        fig = plt.figure()

    ax = fig.add_subplot(111)
    ax.quiver(xs,ys,us,vs,scale=40,width=0.001,pivot='tail')
    ax.set_xlim(base_reachable_set.x_range[0]*1.1,base_reachable_set.x_range[1]*1.1)
    ax.set_ylim(base_reachable_set.y_range[0]*1.1,base_reachable_set.y_range[1]*1.1)
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_title('Dubin\'s Car Base Reachable Set with Turn Radius %.2f m' % base_reachable_set.turn_radius)
    return fig, ax


if __name__=='__main__':
    # load base reachable set
    brs_is_reachable = np.load('../precomputation_results/brs_is_reachables.npy')
    brs_costs = np.load('../precomputation_results/brs_costs.npy')
    base_reachable_set = Base_DC_Reachable_Set(is_reachables=brs_is_reachable,costs=brs_costs)
    visualize_2d_quiver(base_reachable_set)
    plt.show()