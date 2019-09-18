import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.pendulum import Pendulum
from r3t.symbolic_system.symbolic_system_basic_rrt import SymbolicSystem_RGRRT
from pypolycontain.visualization.visualize_2D import visualize_2D_AH_polytope
from pypolycontain.lib.operations import distance_point_polytope
from r3t.utils.visualization import visualize_node_tree_2D
import time
from datetime import datetime
import os
matplotlib.rcParams['font.family'] = "Times New Roman"

global best_distance

def test_pendulum_planning():
    global best_distance
    best_distance = np.inf
    initial_state = np.zeros(2)
    pendulum_system = Pendulum(initial_state= initial_state, input_limits=np.asarray([[-1],[1]]), m=1, l=0.5, g=9.8, b=0.1)
    goal_state = np.asarray([np.pi,0.0])
    goal_state_2 = np.asarray([-np.pi,0.0])
    step_size = 0.08
    def uniform_sampler():
        rnd = np.random.rand(2)
        rnd[0] = (rnd[0]-0.5)*2*1.5*np.pi
        rnd[1] = (rnd[1]-0.5)*2*12
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

    def reached_goal_function(state, goal_state):
        global best_distance
        d1 = np.linalg.norm(state-goal_state)
        d2 = np.linalg.norm(state-goal_state_2)
        best_distance = min(best_distance, min(d1, d2))
        if d1<5e-2 or d2<5e-2:
            print('Goal error %f' %min(np.linalg.norm(state-goal_state), np.linalg.norm(state-goal_state_2)))
            return True
        return False

    rrt = SymbolicSystem_RGRRT(pendulum_system, uniform_sampler, step_size, reached_goal_function=reached_goal_function)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('RG_RRT_Pendulum_'+experiment_name)
    while(1):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state,stop_on_first_reach=True, allocated_time= 100, rewire=False, explore_deterministic_next_state=False) is not None:
            found_goal = True
        end_time = time.time()

        if found_goal:
            p = rrt.goal_node.parent.state
            if np.linalg.norm(p-np.asarray([np.pi,0.0])) < np.linalg.norm(p-np.asarray([-np.pi,0.0])):
                goal_override = np.asarray([np.pi,0.0])
            else:
                goal_override = np.asarray([-np.pi, 0.0])
            rrt.goal_node.children = []
            rrt.goal_node.true_dynamics_path = [p, goal_override]
        # Plot state tree
        print('Best distance %f' %best_distance)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        if found_goal:
            fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=True)#, goal_override=goal_override)
        else:
            fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal)
        # fig, ax = visZ(reachable_polytopes, title="", alpha=0.07, fig=fig,  ax=ax, color='gray')
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        ax.scatter(goal_state[0]-2*np.pi, goal_state[1], facecolor='green', s=5)
        ax.grid(True, which='both')
        y_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
        ax.yaxis.set_major_formatter(y_formatter)
        ax.set_yticks(np.arange(-10, 10, 2))
        ax.set_xlim([-4, 4])
        ax.set_ylim([-10, 10])
        ax.set_xlabel('$\\theta (rad)$')
        ax.set_ylabel('$\dot{\\theta} (rad/s)$')
        duration += (end_time-start_time)
        plt.title('RG-RRT after %.2f seconds (explored %d nodes)' %(duration, rrt.node_tally))
        plt.savefig('RG_RRT_Pendulum_'+experiment_name+'/%.2f_seconds_tree.png' % duration, dpi=500)
        # plt.show()
        plt.xlim([-4, 4])
        plt.ylim([-10,10])
        plt.clf()
        plt.close()

        if found_goal:
            break

if __name__=='__main__':
    for i in range(10):
        test_pendulum_planning()