import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.hopper_1d import Hopper_1d
from rg_rrt_star.symbolic_system.symbolic_system_rg_rrt_star import SymbolicSystem_RGRRTStar
from pypolycontain.visualization.visualize_2D import visualize_2D_zonotopes as visZ
from pypolycontain.lib.AH_polytope import distance_point
from utils.visualization import visualize_node_tree_2D
import time
from datetime import datetime
import os

def test_hopper_planning():
    initial_state = np.asarray([2.,0.])
    l = 1
    p = 0.1
    hopper_system = Hopper_1d(l=l, p=p, initial_state= initial_state, f_max=10)
    goal_state = np.asarray([3,0.0])
    def uniform_sampler():
        rnd = np.random.rand(2)
        rnd[0] = rnd[0]*5
        rnd[1] = (rnd[1]-0.5)*5
        goal_bias = np.random.rand(1)
        if goal_bias<0.1:
            return goal_state
        return rnd

    def gaussian_mixture_sampler():
        gaussian_ratio = 0.4
        rnd = np.random.rand(2)
        rnd[0] = np.random.normal(goal_state[0],1)
        rnd[1] = np.random.normal(goal_state[1],1)
        if np.random.rand(1) > gaussian_ratio:
            return uniform_sampler()
        return rnd

    def contains_goal_function(reachable_set, goal_state):
        distance = np.inf
        projection = None
        for p in reachable_set.polytope_list:
            d, proj = distance_point(p, goal_state)
            if d<distance:
                distance, projection = d, proj
        if abs(projection[0]-goal_state[0])<1e-1 and abs(projection[1]-goal_state[1])<1e-1:
            return True
        return False

    rrt = SymbolicSystem_RGRRTStar(hopper_system, uniform_sampler, 0.05, contains_goal_function=contains_goal_function)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('RRT_Hopper_1d_'+experiment_name)
    while(1):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state,stop_on_first_reach=True, allocated_time= 15, rewire=True) is not None:
            found_goal = True
        end_time = time.time()
        #get rrt polytopes
        polytope_reachable_sets = rrt.reachable_set_tree.id_to_reachable_sets.values()
        reachable_polytopes = []
        explored_states = []
        for prs in polytope_reachable_sets:
            reachable_polytopes.extend(prs.polytope_list)
            explored_states.append(prs.parent_state)
        # print(explored_states)
        # print(len(explored_states))
        # print('number of nodes',rrt.node_tally)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15)
        # fig, ax = visZ(reachable_polytopes, title="", alpha=0.07, fig=fig,  ax=ax, color='gray')
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        # ax.set_aspect('equal')
        plt.plot([l-p, l-p], [-2.5, 2.5], 'm:', lw=1.5)
        plt.plot([l, l], [-2.5, 2.5], 'b:', lw=1.5)

        ax.set_xlim(left=0)
        plt.xlabel('$x$')
        plt.ylabel('$\dot{x}$')
        duration += (end_time-start_time)
        plt.title('RRT Tree after %.2f seconds (explored %d nodes)' %(duration, len(polytope_reachable_sets)))
        plt.savefig('RRT_Hopper_1d_'+experiment_name+'/%.2f_seconds.png' % duration, dpi=500)
        # plt.show()
        plt.clf()
        plt.close()
        if found_goal:
            break


if __name__=='__main__':
    test_hopper_planning()