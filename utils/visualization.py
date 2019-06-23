from collections import deque
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import numpy as np

def visualize_node_tree_2D(rrt, fig=None, ax=None, s=1, linewidths = 0.25):
    if fig is None or ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    node_queue = deque([rrt.root_node])
    lines = []
    i = 0
    while node_queue:
        i+=1
        node = node_queue.popleft()
        if node.children is not None:
            # print(len(node.children))
            node_queue.extend(list(node.children))
            for child in node.children:
                lines.append([np.ndarray.flatten(node.state), np.ndarray.flatten(child.state)])
        ax.scatter(*np.ndarray.flatten(node.state), c='gray', s=s)
    # print('plotted %d nodes' %i)
    lc = mc.LineCollection(lines, linewidths=linewidths, colors='gray')
    ax.add_collection(lc)
    return fig, ax
