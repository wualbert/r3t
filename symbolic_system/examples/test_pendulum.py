import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.pendulum import Pendulum
from rg_rrt_star.symbolic_system.symbolic_system_rg_rrt_star import SymbolicSystem_RGRRTStar
from pypolycontain.visualization.visualize_2D import visualize_2D_AH_polytope
from pypolycontain.lib.operations import distance_point_polytope
from rg_rrt_star.utils.visualization import visualize_node_tree_2D
import time
from datetime import datetime
import os

global best_distance
best_distance=np.inf

def test_pendulum_planning():
    initial_state = np.zeros(2)
    pendulum_system = Pendulum(initial_state= initial_state, input_limits=np.asarray([[-1.1],[1.1]]), m=1, l=0.5, g=9.8, b=0.1)
    goal_state = np.asarray([np.pi,0.0])
    goal_state_2 = np.asarray([-np.pi,0.0])
    step_size = 0.075
    def uniform_sampler():
        rnd = np.random.rand(2)
        rnd[0] = (rnd[0]-0.5)*2*1.5*np.pi
        rnd[1] = (rnd[1]-0.5)*2*9
        goal_bias_rnd = np.random.rand(1)
        # if goal_bias_rnd <0.2:
        #     return goal_state + [2*np.pi*np.random.randint(-1,1),0] + [np.random.normal(0,0.8),np.random.normal(0,1.5)]
        return rnd

    def gaussian_mixture_sampler():
        gaussian_ratio = 0.0
        rnd = np.random.rand(2)
        rnd[0] = np.random.normal(goal_state[0],1)
        rnd[1] = np.random.normal(goal_state[1],1)
        if np.random.rand(1) > gaussian_ratio:
            return uniform_sampler()
        return rnd

    def ring_sampler():
        theta = np.random.rand(1)*2*np.pi
        rnd = np.zeros(2)
        r = np.random.rand(1)+2.5
        rnd[0] = r*np.cos(theta)
        rnd[1] = r*np.sin(theta)
        return rnd

    def big_gaussian_sampler():
        rnd = np.random.rand(2)
        rnd[0] = np.random.normal(0,1.5)
        rnd[1] = np.random.normal(0,3)
        goal_bias_rnd = np.random.rand(1)
        if goal_bias_rnd <0.05:
            return goal_state
        elif goal_bias_rnd < 0.1:
            return np.asarray([-np.pi,0.0])
        return rnd

    def contains_goal_function(reachable_set, goal_state):
        global best_distance
        distance=np.inf
        if np.linalg.norm(reachable_set.parent_state-goal_state)<1:
            distance, projection = distance_point_polytope(reachable_set.polytope_list, goal_state)
        elif np.linalg.norm(reachable_set.parent_state-goal_state_2)<1:
            distance, projection = distance_point_polytope(reachable_set.polytope_list, goal_state_2)
        # # if (abs(projection1[0]-goal_state[0])%(2*np.pi)<3e-1 and abs(projection1[1]-goal_state[1])<3e-1) or \
        # # (abs(projection2[0] - goal_state[0]) % (2 * np.pi) < 3e-1 and abs(projection2[1] - goal_state[1]) < 3e-1):
        # #     return True
        # else:
        #     return False
        # if distance<2e-1:
        # goal_diff = reachable_set.parent_state-goal_state
        # goal_diff[0] %=(2.*np.pi)
        # if np.linalg.norm(reachable_set.parent_state-goal_state)<5e-1 or \
        #         np.linalg.norm(reachable_set.parent_state-goal_state_2)<5e-1:
        #     print(min(np.linalg.norm(reachable_set.parent_state-goal_state),\
        #               np.linalg.norm(reachable_set.parent_state-goal_state_2)))
        if best_distance>distance:
            best_distance=distance
        if distance<1.5e-1:
            # print(goal_diff, np.linalg.norm(goal_diff))
            return True
        return False

    rrt = SymbolicSystem_RGRRTStar(pendulum_system, uniform_sampler, step_size, contains_goal_function=contains_goal_function, use_true_reachable_set=False)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('RRT_Pendulum_'+experiment_name)
    while(1):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state,stop_on_first_reach=True, allocated_time= 10, rewire=True, explore_deterministic_next_state=False) is not None:
            found_goal = True
        end_time = time.time()
        #get rrt polytopes
        polytope_reachable_sets = rrt.reachable_set_tree.id_to_reachable_sets.values()
        reachable_polytopes = []
        explored_states = []
        for prs in polytope_reachable_sets:
            reachable_polytopes.append(prs.polytope_list)
            explored_states.append(prs.parent_state)
        # print(explored_states)
        # print(len(explored_states))
        # print('number of nodes',rrt.node_tally)
        goal_override = None
        if found_goal:
            p = rrt.goal_node.parent.state
            if np.linalg.norm(p-np.asarray([np.pi,0.0])) < np.linalg.norm(p-np.asarray([-np.pi,0.0])):
                goal_override = np.asarray([np.pi,0.0])
            else:
                goal_override = np.asarray([-np.pi, 0.0])

        print('best distance: %f' %best_distance)
        # Plot state tree
        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal, goal_override=goal_override)
        # fig, ax = visZ(reachable_polytopes, title="", alpha=0.07, fig=fig,  ax=ax, color='gray')
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        ax.scatter(goal_state[0]-2*np.pi, goal_state[1], facecolor='green', s=5)

        # ax.set_aspect('equal')
        plt.xlabel('$\\theta$')
        plt.ylabel('$\dot{\\theta}$')
        duration += (end_time-start_time)
        plt.title('RRT Tree after %.2f seconds (explored %d nodes)' %(duration, len(polytope_reachable_sets)))
        plt.savefig('RRT_Pendulum_'+experiment_name+'/%.2f_seconds_tree.png' % duration, dpi=500)
        # plt.show()
        plt.xlim([-4, 4])
        plt.ylim([-10,10])
        plt.clf()
        plt.close()


        # # Plot explored reachable sets
        # FIXME: Handle degenerated reachable set
        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_2D_AH_polytope(reachable_polytopes, fig=fig, ax=ax,N=200,epsilon=0.01)

        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        ax.scatter(goal_state[0]-2*np.pi, goal_state[1], facecolor='green', s=5)

        # ax.set_aspect('equal')
        plt.xlabel('$x$')
        plt.ylabel('$\dot{x}$')
        duration += (end_time-start_time)
        plt.title('RRT Tree after %.2f seconds (explored %d nodes)' %(duration, len(polytope_reachable_sets)))
        plt.savefig('RRT_Pendulum_'+experiment_name+'/%.2f_seconds_reachable_sets.png' % duration, dpi=500)
        # plt.show()
        plt.clf()
        # plt.close()

        if found_goal:
            break

if __name__=='__main__':
    for i in range(10):
        test_pendulum_planning()