import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import pickle
import os
import dubins

from dubin_car.dubin_car_rg_rrt_star import *

def visualize_tree(dc_rrt, world_map, visualize_all_nodes=True, visualize_all_paths=False, visualize_goal_path=True, visualize_obstacles=True, ax = None, fig=None):
    if ax is None or fig is None:
        fig, ax = plt.subplots()
    #visualize start
    ax.quiver([dc_rrt.root_node.state[0]],[dc_rrt.root_node.state[1]],
              [np.cos(dc_rrt.root_node.state[2])],[np.sin(dc_rrt.root_node.state[2])],facecolor='r',
              scale=40,width=0.002,pivot='mid')
    #visualize goal
    if dc_rrt.goal_state is not None:
        ax.quiver([dc_rrt.goal_state[0]], [dc_rrt.goal_state[1]],
                  [np.cos(dc_rrt.goal_state[2])], [np.sin(dc_rrt.goal_state[2])], facecolor='g',
                  scale=40, width=0.002, pivot='mid')
    #visualize nodes and edges
    node_queue =deque()
    node_queue.append(dc_rrt.root_node)
    # print('root', len(dc_rrt.root_node.children))
    if visualize_obstacles:
        visualize_boxes(world_map.obstacles, ax=ax, fig=fig, fill=True, alpha = 0.3)

    if visualize_all_nodes or visualize_all_paths:
        tree_path_segments = []
        while(len(node_queue)>0):
            #pop from the top of the queue
            node = node_queue.popleft()
            #get the children of this node and add to the queue
            children_nodes = node.children
            node_queue.extend(children_nodes)
            #visualize the state
            if visualize_all_nodes:
                ax.quiver([node.state[0]], [node.state[1]],
                          [np.cos(node.state[2])], [np.sin(node.state[2])], facecolor='k',
                          scale=40, width=0.002, pivot='mid', alpha=0.4)
            if visualize_all_paths and node.parent is not None and node.path_from_parent is not None:
                #SLOW!
                #reconstruct dubin's path on the fly
                segs = node.path_from_parent.get_dubins_interpolated_path()
                for i in range(segs.shape[0]-1):
                    tree_path_segments.append([segs[i,0:2], segs[i+1,0:2]])
        if visualize_all_paths:
            tree_lc = mc.LineCollection(tree_path_segments, colors='k', linewidths=0.2, alpha=0.5)
            ax.add_collection(tree_lc)
    if visualize_goal_path:
        node = dc_rrt.goal_node
        goal_path_segments = []
        #Green goal node
        segs = node.path_from_parent.get_dubins_interpolated_path()
        for i in range(segs.shape[0] - 1):
            goal_path_segments.append([segs[i, 0:2], segs[i + 1, 0:2]])
        ax.quiver([node.state[0]], [node.state[1]],
                  [np.cos(node.state[2])], [np.sin(node.state[2])], facecolor='g',
                  scale=40, width=0.002, pivot='mid', alpha=1)
        node = node.parent
        while node.parent!=None:
            segs = node.path_from_parent.get_dubins_interpolated_path()
            for i in range(segs.shape[0] - 1):
                goal_path_segments.append([segs[i, 0:2], segs[i + 1, 0:2]])
            ax.quiver([node.state[0]], [node.state[1]],
                      [np.cos(node.state[2])], [np.sin(node.state[2])], facecolor='b',
                      scale=40, width=0.002, pivot='mid', alpha=1)
            node = node.parent

        goal_lc = mc.LineCollection(goal_path_segments, colors='b', linewidths=1,alpha=0.7)
        ax.add_collection(goal_lc)
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    plt.xlim(world_map.world_bounding_box.u[0],world_map.world_bounding_box.v[0])
    plt.ylim(world_map.world_bounding_box.u[1],world_map.world_bounding_box.v[1])
    return fig, ax