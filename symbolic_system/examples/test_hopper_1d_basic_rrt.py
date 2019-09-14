import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.hopper_1d import Hopper_1d
from r3t.symbolic_system.symbolic_system_basic_rrt import SymbolicSystem_Basic_RRT
from pypolycontain.visualization.visualize_2D import visualize_2D_zonotopes as visZ
from pypolycontain.lib.operations import distance_point_polytope
from r3t.utils.visualization import visualize_node_tree_2D_old
import time
from datetime import datetime
import os
matplotlib.rcParams['font.family'] = "Times New Roman"
matplotlib.rcParams.update({'font.size': 14})

global best_distance

def test_hopper_1d_planning():
    global best_distance
    best_distance=np.inf

    initial_state = np.asarray([2.,0.])
    l = 1
    p = 0.1
    step_size = 0.01
    hopper_system = Hopper_1d(l=l, p=p, initial_state= initial_state)
    goal_state = np.asarray([3,0.0])
    goal_tolerance = 5e-2
    def uniform_basic_sampler():
        rnd = np.random.rand(2)
        rnd[0] = rnd[0] * 5-.5
        rnd[1] = (rnd[1] - 0.5) * 4 * 5
        goal_bias = np.random.rand(1)
        if goal_bias < 0.25:
            return goal_state
        return rnd

    def uniform_sampler():
        rnd = np.random.rand(2)
        rnd[0] = rnd[0]*5+0.5
        rnd[1] = (rnd[1]-0.5)*2*10
        goal_bias = np.random.rand(1)
        if goal_bias<0.1:
            return goal_state
        return rnd

    def gaussian_mixture_sampler():
        gaussian_ratio = 0.4
        rnd = np.zeros(2)
        rnd[0] = np.random.normal(l+0.5*p,3*p)
        rnd[1] = (np.random.rand(1)-0.5)*2*10
        if np.random.rand(1) > gaussian_ratio:
            return uniform_sampler()
        return rnd

    def action_space_mixture_sampler():
        action_space_ratio = 0.08
        if np.random.rand(1) < action_space_ratio:
            rnd = np.random.rand(2)
            rnd[0] = rnd[0]*p*1.2+l
            rnd[1] = (rnd[1]-0.5)*2*8
            return rnd
        else:
            rnd = np.random.rand(2)
            rnd[0] = rnd[0]*4 + l
            rnd[1] = (rnd[1] - 0.5) * 2 * 2
            goal_bias = np.random.rand(1)
            if goal_bias < 0.35:
                rnd[0] = np.random.normal(goal_state[0],1.5)
                rnd[1] = np.random.normal(goal_state[1],1.5)
                return rnd
            return rnd

    def reached_goal_function(state, goal_state):
        global best_distance
        distance = np.linalg.norm(state-goal_state)
        best_distance = min(distance, best_distance)
        if distance<goal_tolerance:
            print(distance, goal_tolerance)
            return True
        return False

    def contains_goal_function(reachable_set, goal_state):
        distance = np.inf
        projection = None
        # reachable set is a singular point
        # if reahc
        #
        # else:
        for i, p in enumerate(reachable_set.polytope_list):
            if (p.T==0.).all():
                # reachable set is a line
                # check if the goal is on the line
                vec_to_goal = goal_state - reachable_set.parent_state
                vec_to_reachable_set = np.ndarray.flatten(p.t)-reachable_set.parent_state
                # normalize
                vec_to_reachable_set_norm = np.linalg.norm(vec_to_reachable_set)
                vec_dot = np.dot(vec_to_reachable_set, vec_to_goal)/vec_to_reachable_set_norm
                crosstrack_vec = vec_to_goal-vec_dot*vec_to_reachable_set/vec_to_reachable_set_norm
                if np.linalg.norm(crosstrack_vec)<distance and 0.<=vec_dot<=vec_to_reachable_set_norm:
                    distance = np.linalg.norm(crosstrack_vec)
            else:
                d, proj = distance_point_polytope(p, goal_state)
                if d<distance:
                    projection = proj
                    distance = d
        if distance<goal_tolerance:
            print(distance, goal_tolerance)
            return True
        return False

    rrt = SymbolicSystem_Basic_RRT(hopper_system, gaussian_mixture_sampler, step_size, reached_goal_function=reached_goal_function)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('Basic_RRT_Hopper_1d_'+experiment_name)
    max_iterations = 1
    for itr in range(max_iterations):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state, stop_on_first_reach=True, allocated_time= 100, rewire=True, explore_deterministic_next_state=True) is not None:
            found_goal = True
        end_time = time.time()
        print("Best distance %f" %best_distance)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_2D_old(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal)
        # fig, ax = visZ(reachable_polytopes, title="", alpha=0.07, fig=fig,  ax=ax, color='gray')
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        # ax.set_aspect('equal')
        plt.plot([l+p, l+p], [-7,7], 'm--', lw=1.5)
        plt.plot([l, l], [-7,7], 'b--', lw=1.5)
        plt.xlim([0.5,4.5])
        plt.ylim([-8,8])
        plt.tight_layout()
        plt.gcf().subplots_adjust(left=0.13, bottom=0.12)
        # ax.set_xlim(left=0)
        ax.grid(True, which='both')
        # ax.set_xlim(left=0)
        plt.xlabel('$x(m)$')
        plt.ylabel('$\dot{x}(m/s)$')
        duration += (end_time-start_time)
        plt.title('RRT after %.2f seconds (explored %d nodes)' %(duration, rrt.node_tally))
        plt.savefig('Basic_RRT_Hopper_1d_'+experiment_name+'/%.2f_seconds.png' % duration, dpi=500)
        # plt.show()
        plt.clf()
        plt.close()
        if found_goal:
            break


if __name__=='__main__':
    for i in range(1):
        test_hopper_1d_planning()