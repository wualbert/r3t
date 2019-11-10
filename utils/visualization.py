from collections import deque
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import numpy as np
from r3t.symbolic_system.examples.hopper_2D_visualize import *
import pypolycontain.visualization.visualize_2D as vis2D

def visualize_node_tree_2D(rrt, fig=None, ax=None, s=1, linewidths = 0.25, show_path_to_goal=False, goal_override=None, dims=[0,1]):
    if fig is None or ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    node_queue = deque([rrt.root_node])
    lines = []
    i = 0
    while node_queue:
        i+=1
        node = node_queue.popleft()
        #handle indexing
        if dims:
            state = np.ndarray.flatten(node.state)[dims]
        else:
            state = np.ndarray.flatten(node.state)
        #handle goal
        if goal_override is not None and node==rrt.goal_node:
            lines.append([state, goal_override])
        elif node == rrt.root_node or node==rrt.goal_node:
            pass
        else:
            for i in range(len(node.true_dynamics_path)-1):
                # handle indexing
                if dims:
                    lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[dims],
                                   np.ndarray.flatten(node.true_dynamics_path[i + 1])[dims]])
                else:
                    lines.append([np.ndarray.flatten(node.true_dynamics_path[i]),
                                  np.ndarray.flatten(node.true_dynamics_path[i + 1])])

        if node.children is not None:
            # print(len(node.children))
            node_queue.extend(list(node.children))
        ax.scatter(*state, c='gray', s=s)
    if show_path_to_goal:
        goal_lines = []
        node = rrt.goal_node
        if goal_override is not None:
            #FIXME: make this cleaner
            goal_lines.append([goal_override[dims], np.ndarray.flatten(node.parent.state)[dims]])
            node = node.parent
        else:
            goal_lines.append([rrt.goal_node.state[dims], np.ndarray.flatten(node.parent.state)[dims]])
            node = node.parent
        while node.parent is not None:
            for i in range(len(node.true_dynamics_path)-1):
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[dims], np.ndarray.flatten(node.true_dynamics_path[i+1])[dims]])
            assert(node in node.parent.children)
            # hack for 1D hopper visualization
            if node.parent==rrt.root_node:
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[-1])[dims], np.ndarray.flatten(node.parent.state)[dims]])
            node = node.parent
        line_colors = np.full(len(lines), 'gray')
        line_widths = np.full(len(lines), linewidths)
        goal_line_colors = np.full(len(goal_lines), 'cyan')
        goal_line_widths= np.full(len(goal_lines), linewidths*4)
        lines.extend(goal_lines)
        all_colors = np.hstack([line_colors, goal_line_colors])
        all_widths = np.hstack([line_widths, goal_line_widths])
        lc = mc.LineCollection(lines, linewidths=all_widths, colors=all_colors)
        ax.add_collection(lc)
    else:
        lc = mc.LineCollection(lines, linewidths=linewidths, colors='gray')
        ax.add_collection(lc)
    return fig, ax

def visualize_node_tree_2D_old(rrt, fig=None, ax=None, s=1, linewidths = 0.25, show_path_to_goal=False, goal_override=None, dims=None):
    """
    Deprecated function for visualizing RRT trees with no true dynamics path (prior to 9/9/19)
    :param rrt:
    :param fig:
    :param ax:
    :param s:
    :param linewidths:
    :param show_path_to_goal:
    :param goal_override:
    :param dims:
    :return:
    """
    if fig is None or ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    node_queue = deque([rrt.root_node])
    lines = []
    i = 0
    while node_queue:
        i+=1
        node = node_queue.popleft()
        if dims:
            state = np.ndarray.flatten(node.state)[dims]
        else:
            state = np.ndarray.flatten(node.state)
        if node.children is not None:
            # print(len(node.children))
            node_queue.extend(list(node.children))
            for child in node.children:
                if dims:
                    child_state = np.ndarray.flatten(child.state)[dims]
                else:
                    child_state = np.ndarray.flatten(child.state)
                # don't plot the goal if goal override is on
                if goal_override is not None and child==rrt.goal_node:
                    lines.append([state, goal_override])
                else:
                    lines.append([state, child_state])
        ax.scatter(*state, c='gray', s=s)
    if show_path_to_goal:
        goal_lines = []
        node = rrt.goal_node
        if goal_override is not None:
            if dims:
                goal_lines.append([goal_override[dims], np.ndarray.flatten(node.parent.state)[dims]])
            else:
                goal_lines.append([goal_override, np.ndarray.flatten(node.parent.state)])
            node = node.parent
        while node.parent is not None:
            for i in range(len(node.true_dynamics_path)-1):
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[dims], np.ndarray.flatten(node.true_dynamics_path[i+1])[dims]])
            assert(node in node.parent.children)
            # hack for 1D hopper visualization
            if node.parent==rrt.root_node:
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[-1])[dims], np.ndarray.flatten(node.parent.state)[dims]])

            node = node.parent
        line_colors = np.full(len(lines), 'gray')
        line_widths = np.full(len(lines), linewidths)
        goal_line_colors = np.full(len(goal_lines), 'cyan')
        goal_line_widths= np.full(len(goal_lines), linewidths*4)
        lines.extend(goal_lines)
        all_colors = np.hstack([line_colors, goal_line_colors])
        all_widths = np.hstack([line_widths, goal_line_widths])
        lc = mc.LineCollection(lines, linewidths=all_widths, colors=all_colors)
        ax.add_collection(lc)
    else:
        lc = mc.LineCollection(lines, linewidths=linewidths, colors='gray')
        ax.add_collection(lc)
    return fig, ax

def visualize_node_tree_hopper_2D(rrt, fig=None, ax=None, s=1, linewidths = 0.25, show_path_to_goal=False, goal_override=None,\
                                  dims=[0,1], show_body_attitude='goal', scaling_factor=1, draw_goal =False, ground_height_function = None, downsample=3):
    """

    :param rrt:
    :param fig:
    :param ax:
    :param s:
    :param linewidths:
    :param show_path_to_goal:
    :param goal_override:
    :param dims:
    :param show_body_attitude: 'goal', 'all', or nothing
    :param scaling_factor:
    :param draw_goal:
    :param ground_height_function:
    :return:
    """
    if fig is None or ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    node_queue = deque([rrt.root_node])
    lines = []
    i = 0
    nodes_to_visualize = [rrt.root_node]

    while node_queue:
        i+=1
        node = node_queue.popleft()
        #handle indexing
        if dims:
            state = np.ndarray.flatten(node.state)[dims]
        else:
            state = np.ndarray.flatten(node.state)
        #handle goal
        if goal_override is not None and node==rrt.goal_node:
            lines.append([state, goal_override])
        elif node==rrt.goal_node:
            pass
        else:
            for i in range(len(node.true_dynamics_path)-1):
                # handle indexing
                if dims:
                    lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[dims],
                                   np.ndarray.flatten(node.true_dynamics_path[i + 1])[dims]])
                else:
                    lines.append([np.ndarray.flatten(node.true_dynamics_path[i]),
                                  np.ndarray.flatten(node.true_dynamics_path[i + 1])])
        if node.parent == rrt.root_node:
            lines.append([np.ndarray.flatten(node.parent.state)[dims],
                          np.ndarray.flatten(node.true_dynamics_path[0])[dims]])
        if node.children is not None:
            # print(len(node.children))
            nodes_to_visualize.extend(list(node.children))
            node_queue.extend(list(node.children))
        ax.scatter(*state, c='gray', s=s)

    #
    # while node_queue:
    #     i+=1
    #     node = node_queue.popleft()
    #     #handle indexing
    #     if dims:
    #         state = np.ndarray.flatten(node.state)[dims]
    #     else:
    #         state = np.ndarray.flatten(node.state)
    #     for i in range(len(node.true_dynamics_path) - 1):
    #         # handle indexing
    #         if dims:
    #             lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[dims],
    #                           np.ndarray.flatten(node.true_dynamics_path[i + 1])[dims]])
    #         else:
    #             lines.append([np.ndarray.flatten(node.true_dynamics_path[i]),
    #                           np.ndarray.flatten(node.true_dynamics_path[i + 1])])
    #
    #     if node.children is not None:
    #         # print(len(node.children))
    #         node_queue.extend(list(node.children))
    #         nodes_to_visualize.extend(list(node.children))
    #         for child in node.children:
    #             if dims:
    #                 child_state = np.ndarray.flatten(child.state)[dims]
    #             else:
    #                 child_state = np.ndarray.flatten(child.state)
    #             # # don't plot the goal if goal override is on
    #             # if goal_override is not None and child==rrt.goal_node:
    #             #     lines.append([state, goal_override])
    #             # else:
    #             #     lines.append([state, child_state])
    #     ax.scatter(*state, c='gray', s=s)

    if show_path_to_goal:
        goal_lines = []
        node = rrt.goal_node
        if not draw_goal:
            node = node.parent
        if goal_override is not None:
            goal_lines.append([goal_override[dims], np.ndarray.flatten(node.parent.state)[dims]])
        while node.parent is not None:
            for i in range(len(node.true_dynamics_path)-1):
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[dims], np.ndarray.flatten(node.true_dynamics_path[i+1])[dims]])
            assert(node in node.parent.children)
            # hack for 1D hopper visualization
            if node.parent==rrt.root_node:
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[-1])[dims], np.ndarray.flatten(node.parent.state)[dims]])
            node = node.parent
        line_colors = np.full(len(lines), 'gray')
        line_widths = np.full(len(lines), linewidths)
        goal_line_colors = np.full(len(goal_lines), 'cyan')
        goal_line_widths= np.full(len(goal_lines), linewidths*4)
        lines.extend(goal_lines)
        all_colors = np.hstack([line_colors, goal_line_colors])
        all_widths = np.hstack([line_widths, goal_line_widths])
        lc = mc.LineCollection(lines, linewidths=all_widths, colors=all_colors)
        ax.add_collection(lc)
    else:
        lc = mc.LineCollection(lines, linewidths=linewidths, colors='gray')
        ax.add_collection(lc)
    if show_body_attitude =='goal':
        node = rrt.goal_node
        skip = downsample
        if not draw_goal and node is not None:
            node = node.parent
        while node is not None:
            if node.parent is None:
                #reached root
                #plot root
                fig, ax = hopper_plot(node.state, fig, ax, alpha=0.2, scaling_factor=scaling_factor)
                node = node.parent
                break
            if skip<downsample:
                #skipping
                node = node.parent
                skip+=1
            else:
                #plot
                fig, ax = hopper_plot(node.state, fig, ax, alpha=0.25, scaling_factor=scaling_factor)
                skip = 0
    elif show_body_attitude=='all':
        for i, n in enumerate(nodes_to_visualize):
            # fig, ax = hopper_plot(n.state, fig, ax, alpha=0.5/len(nodes_to_visualize)*i+0.1)
            fig, ax = hopper_plot(n.state, fig, ax, alpha=0.15, scaling_factor=scaling_factor)
    return fig, ax

def visualize_projected_ND_polytopic_tree(rrt, dim1, dim2, fig=None, ax=None, s=10, linewidths =1., show_path_to_goal=False, goal_override=None, polytope_alpha=0.06):
    if fig is None or ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    node_queue = deque([rrt.root_node])
    lines = []
    i = 0
    polytopes_list = []
    while node_queue:
        i+=1
        node = node_queue.popleft()
        # give a separate color for all polytopes in one node
        random_color = np.random.rand(3)
        for p in node.reachable_set.polytope_list:
            p.color = random_color
            polytopes_list.append(p)
        #handle indexing
        state = np.ndarray.flatten(node.state)[np.asarray([dim1, dim2])]
        # plot the polytope
        if goal_override is not None and node==rrt.goal_node:
            lines.append([state, goal_override])
        elif node == rrt.root_node or node==rrt.goal_node:
            pass
        else:
            try:
                for i in range(len(node.true_dynamics_path) - 1):
                    # handle indexing
                    lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[np.asarray([dim1, dim2])],
                                  np.ndarray.flatten(node.true_dynamics_path[i + 1])[np.asarray([dim1, dim2])]])
            except:
                lines.append([np.ndarray.flatten(node.path_from_parent.state_from)[np.asarray([dim1, dim2])],
                              np.ndarray.flatten(node.path_from_parent.state_to)[np.asarray([dim1, dim2])]])
        if node.children is not None:
            # print(len(node.children))
            node_queue.extend(list(node.children))
        ax.scatter(*state, c='black', s=s, alpha=1)
    if show_path_to_goal and rrt.goal_node:
        goal_lines = []
        node = rrt.goal_node
        if goal_override is not None:
            #FIXME: make this cleaner
            goal_lines.append([goal_override[np.asarray([dim1, dim2])], np.ndarray.flatten(node.parent.state)[np.asarray([dim1, dim2])]])
            node = node.parent
        else:
            goal_lines.append([rrt.goal_node.state[np.asarray([dim1, dim2])], np.ndarray.flatten(node.parent.state)[np.asarray([dim1, dim2])]])
            node = node.parent
        while node.parent is not None:
            for i in range(len(node.true_dynamics_path)-1):
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[i])[np.asarray([dim1, dim2])], np.ndarray.flatten(node.true_dynamics_path[i+1])[np.asarray([dim1, dim2])]])
            assert(node in node.parent.children)
            # hack for 1D hopper visualization
            if node.parent==rrt.root_node:
                goal_lines.append([np.ndarray.flatten(node.true_dynamics_path[-1])[np.asarray([dim1, dim2])], np.ndarray.flatten(node.parent.state)[np.asarray([dim1, dim2])]])
            node = node.parent
        line_colors = np.full(len(lines), 'gray', alpha=1)
        line_widths = np.full(len(lines), linewidths)
        goal_line_colors = np.full(len(goal_lines), 'cyan')
        goal_line_widths= np.full(len(goal_lines), linewidths*4)
        lines.extend(goal_lines)
        all_colors = np.hstack([line_colors, goal_line_colors])
        all_widths = np.hstack([line_widths, goal_line_widths])
        lc = mc.LineCollection(lines, linewidths=all_widths, colors=all_colors)
        ax.add_collection(lc)
    else:
        lc = mc.LineCollection(lines, linewidths=linewidths, colors='black', alpha=1)
        ax.add_collection(lc)

    vis2D.visualize_ND_AH_polytope(polytopes_list, dim1, dim2, fig=fig, ax=ax, alpha=polytope_alpha)

    return fig, ax