import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.hopper_2d import Hopper_2d
from rg_rrt_star.symbolic_system.symbolic_system_rg_rrt_star import SymbolicSystem_RGRRTStar
from pypolycontain.visualization.visualize_2D import visualize_2D_zonotopes as visZ
from pypolycontain.lib.operations import distance_point_polytope
from rg_rrt_star.utils.visualization import visualize_node_tree_2D
import time
from datetime import datetime
import os

def test_hopper_2d_planning():
    initial_state = np.asarray([0.,0.,0.,1.,0.8,0.,0.,0.,0.,0.])
    hopper_system = Hopper_2d(initial_state=initial_state)
    # [theta1, theta2, x0, y0, w]
    # from x0 = 0 move to x0 = 5
    goal_state = np.asarray([0.,0.,50.,0.,0.,0.,0.,0.,0.,0.])
    goal_tolerance = [10,10,2.5e-2,10,10,5,5,5,5,5]
    step_size = 5e-2
    #TODO
    def uniform_sampler():
        rnd = np.random.rand(10)
        rnd[0] = (rnd[0]-0.5)*2*np.pi
        rnd[1] = (rnd[1] - 0.5) * 2 * np.pi
        rnd[2] = (rnd[2] - 0.5) * 2 * 150
        rnd[3] = rnd[3] * 20
        rnd[5:] = (rnd[5:]-0.5)*2*5
        goal_bias = np.random.rand(1)
        return rnd

    # def gaussian_mixture_sampler():
    #     gaussian_ratio = 0.4
    #     rnd = np.random.rand(2)
    #     rnd[0] = np.random.normal(l+0.5*p,2*p)
    #     rnd[1] = (np.random.rand(1)-0.5)*2*4
    #     if np.random.rand(1) > gaussian_ratio:
    #         return uniform_sampler()
    #     return rnd
    #
    # def action_space_mixture_sampler():
    #     action_space_ratio = 0.08
    #     if np.random.rand(1) < action_space_ratio:
    #         rnd = np.random.rand(2)
    #         rnd[0] = rnd[0]*p*1.2+l
    #         rnd[1] = (rnd[1]-0.5)*2*8
    #         return rnd
    #     else:
    #         rnd = np.random.rand(2)
    #         rnd[0] = rnd[0]*4 + l
    #         rnd[1] = (rnd[1] - 0.5) * 2 * 2
    #         goal_bias = np.random.rand(1)
    #         if goal_bias < 0.35:
    #             rnd[0] = np.random.normal(goal_state[0],1.5)
    #             rnd[1] = np.random.normal(goal_state[1],1.5)
    #             return rnd
    #         return rnd

    def contains_goal_function(reachable_set, goal_state):
        distance = np.inf
        projection = None
        for p in reachable_set.polytope_list:
            d, proj = distance_point_polytope(p, goal_state)
            if d<distance:
                projection = np.asarray(proj)
                distance = d
        if np.all(abs(projection-goal_state)<goal_tolerance):
            return True
        return False

    rrt = SymbolicSystem_RGRRTStar(hopper_system, uniform_sampler, step_size, contains_goal_function=contains_goal_function)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('RRT_Hopper_2d_'+experiment_name)
    max_iterations = 100
    for itr in range(max_iterations):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state, stop_on_first_reach=True, allocated_time= 15, rewire=True, explore_deterministic_next_state=True) is not None:
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
        fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal, dims=[2,3])
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[2], initial_state[3], facecolor='red', s=5)
        # ax.set_aspect('equal')
        # plt.plot([l+p, l+p], [-2.5, 2.5], 'm--', lw=1.5)
        plt.plot([5,5], [-2.5, 2.5], 'g--', lw=1.5)

        # ax.set_xlim(left=0)
        plt.xlabel('$x_0$')
        plt.ylabel('$y_0$')
        duration += (end_time-start_time)
        plt.title('RRT Tree after %.2f seconds (explored %d nodes)' %(duration, len(polytope_reachable_sets)))
        plt.savefig('RRT_Hopper_2d_'+experiment_name+'/%.2f_seconds.png' % duration, dpi=500)
        # plt.show()
        plt.clf()
        plt.close()
        if found_goal:
            break


if __name__=='__main__':
    test_hopper_2d_planning()