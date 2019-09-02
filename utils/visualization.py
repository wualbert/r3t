from collections import deque
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import numpy as np

def visualize_node_tree_2D(rrt, fig=None, ax=None, s=1, linewidths = 0.25, show_path_to_goal=False, goal_override=None, dims=None):
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
    # print('plotted %d nodes' %i)
    if show_path_to_goal:
        goal_lines = []
        node = rrt.goal_node
        if goal_override is not None:
            goal_lines.append([goal_override, np.ndarray.flatten(node.parent.state)])
            node = node.parent
        while node.parent is not None:
            goal_lines.append([np.ndarray.flatten(node.state), np.ndarray.flatten(node.parent.state)])
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
