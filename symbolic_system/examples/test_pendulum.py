import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.pendulum import Pendulum
from r3t.symbolic_system.symbolic_system_r3t import SymbolicSystem_R3T
from pypolycontain.visualization.visualize_2D import visualize_2D_AH_polytope
from pypolycontain.lib.operations import distance_point_polytope
from r3t.utils.visualization import visualize_node_tree_2D
import time
from datetime import datetime
import os
matplotlib.rcParams['font.family'] = "Times New Roman"
matplotlib.rcParams.update({'font.size': 14})

reachable_set_epsilon = 2
goal_tolerance = 5e-2
input_limit = 1
input_samples = 9

def test_pendulum_planning():
    initial_state = np.zeros(2)

    pendulum_system = Pendulum(initial_state= initial_state, input_limits=np.asarray([[-input_limit],[input_limit]]), m=1, l=0.5, g=9.8, b=0.1)
    goal_state = np.asarray([np.pi,0.0])
    goal_state_2 = np.asarray([-np.pi,0.0])
    step_size = 0.1 # 0.075
    nonlinear_dynamic_step_size=1e-2
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

    def contains_goal_function(reachable_set, goal_state):
        distance=np.inf
        if np.linalg.norm(reachable_set.parent_state-goal_state)<2:
            distance, projection = distance_point_polytope(reachable_set.polytope_list, goal_state)
        elif np.linalg.norm(reachable_set.parent_state-goal_state_2)<2:
            distance, projection = distance_point_polytope(reachable_set.polytope_list, goal_state_2)
        else:
            return False, None
        if distance > reachable_set_epsilon:
            return False, None
        #enumerate inputs
        potential_inputs = np.linspace(pendulum_system.input_limits[0,0], pendulum_system.input_limits[1,0], input_samples)
        for u_i in potential_inputs:
            state_list = []
            state=reachable_set.parent_state
            for step in range(int(step_size/nonlinear_dynamic_step_size)):
                state = pendulum_system.forward_step(u=np.atleast_1d(u_i), linearlize=False, modify_system=False, step_size = nonlinear_dynamic_step_size, return_as_env = False,
                     starting_state= state)
                state_list.append(state)
                if np.linalg.norm(goal_state-state)<goal_tolerance:
                    print('Goal error is %d' % np.linalg.norm(goal_state-state))
                    return True, np.asarray(state_list)
                if np.linalg.norm(goal_state_2-state)<goal_tolerance:
                    print('Goal error is %d' % np.linalg.norm(goal_state_2-state))
                    return True, np.asarray(state_list)
        return False, None

    rrt = SymbolicSystem_R3T(pendulum_system, uniform_sampler, step_size, contains_goal_function=contains_goal_function, \
                             use_true_reachable_set=True, use_convex_hull=True)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('R3T_Pendulum_'+experiment_name)
    allocated_time = 0.1
    while(1):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state,stop_on_first_reach=True, allocated_time= allocated_time, rewire=False, explore_deterministic_next_state=False, save_true_dynamics_path=True) is not None:
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
        # ax.grid(True, which='both')
        # y_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
        # ax.yaxis.set_major_formatter(y_formatter)
        # ax.set_yticks(np.arange(-10, 10, 2))
        # ax.set_xlim([-4, 4])
        # ax.set_ylim([-10, 10])
        # ax.set_xlabel('$\\theta (rad)$')
        # ax.set_ylabel('$\dot{\\theta} (rad/s)$')
        #
        duration += (end_time-start_time)
        # plt.title('R3T after %.2f seconds (explored %d nodes)' %(duration, len(polytope_reachable_sets)))
        # plt.savefig('R3T_Pendulum_'+experiment_name+'/%.2f_seconds_tree.png' % duration, dpi=500)
        # # plt.show()
        # plt.xlim([-4, 4])
        # plt.ylim([-10,10])
        # plt.clf()
        # plt.close()

        #
        # # # Plot explored reachable sets
        # # FIXME: Handle degenerated reachable set
        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        fig, ax = visualize_2D_AH_polytope(reachable_polytopes, fig=fig, ax=ax,N=200,epsilon=0.01, alpha=0.1)

        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        ax.scatter(goal_state[0]-2*np.pi, goal_state[1], facecolor='green', s=5)

        # ax.set_aspect('equal')
        plt.xlabel('$x$')
        plt.ylabel('$\dot{x}$')
        plt.xlim([-5, 5])
        plt.ylim([-12,12])
        plt.tight_layout()
        plt.title('$|u| \leq %.2f$ Reachable Set after %.2fs (%d nodes)' %(input_limit, duration, len(polytope_reachable_sets)))
        plt.savefig('R3T_Pendulum_'+experiment_name+'/%.2f_seconds_reachable_sets.png' % duration, dpi=500)
        # plt.show()
        plt.clf()
        plt.close()
        #
        # if found_goal:
        #     break
        # allocated_time*=5
if __name__=='__main__':
    for i in range(1):
        test_pendulum_planning()