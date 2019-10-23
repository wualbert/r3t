import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.hopper_1d import Hopper_1d
from r3t.symbolic_system.symbolic_system_r3t import SymbolicSystem_R3T
from pypolycontain.visualization.visualize_2D import visualize_2D_AH_polytope
from pypolycontain.lib.operations import distance_point_polytope
from r3t.utils.visualization import visualize_node_tree_2D
import time
from datetime import datetime
import os
matplotlib.rcParams['font.family'] = "Times New Roman"
matplotlib.rcParams.update({'font.size': 14})
input_samples = 9
nonlinear_dynamic_step_size = 1e-2

global best_distance
def test_hopper_1d_planning():
    global best_distance
    best_distance=np.inf
    initial_state = np.asarray([2.,0.])
    l = 1
    p = 0.1
    step_size = 4e-2
    hopper_system = Hopper_1d(l=l, p=p, initial_state= initial_state)
    goal_state = np.asarray([3,0.0])
    goal_tolerance = 5e-2
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

    def flight_sampler():
        rnd = np.random.rand(2)
        rnd[0] = rnd[0]*5+l+p
        rnd[1] = (rnd[1]-0.5)*2*10
        goal_bias = np.random.rand(1)
        return rnd


    def action_space_mixture_sampler():
        action_space_ratio = 0.9
        if np.random.rand(1) < action_space_ratio:
            rnd = np.random.rand(2)
            rnd[0] = rnd[0]*p*1.2+l
            rnd[1] = (rnd[1]-0.5)*2*8
            return rnd
        else:
            # rnd = np.random.rand(2)
            # rnd[0] = rnd[0]*4 + l
            # rnd[1] = (rnd[1] - 0.5) * 2 * 2
            # goal_bias = np.random.rand(1)
            # if goal_bias < 0.35:
            #     rnd[0] = np.random.normal(goal_state[0],1.5)
            #     rnd[1] = np.random.normal(goal_state[1],1.5)
            #     return rnd
            return uniform_sampler()

    # def contains_goal_function(reachable_set, goal_state):
    #     distance = np.linalg.norm(reachable_set.parent_state-goal_state)
    #     if distance<goal_tolerance:
    #         print(distance, goal_tolerance)
    #         return True
    #     return False

    def contains_goal_function(reachable_set, goal_state):
        global best_distance
        distance = np.inf
        projection = None
        # reachable set is a singular point
        for i, p in enumerate(reachable_set.polytope_list):
            # if (p.T==0.).all():
            #     # reachable set is a line
            #     # check if the goal is on the line
            #     vec_to_goal = goal_state - reachable_set.parent_state
            #     vec_to_reachable_set = np.ndarray.flatten(p.t)-reachable_set.parent_state
            #     # normalize
            #     vec_to_reachable_set_norm = np.linalg.norm(vec_to_reachable_set)
            #     vec_dot = np.dot(vec_to_reachable_set, vec_to_goal)/vec_to_reachable_set_norm
            #     crosstrack_vec = vec_to_goal-vec_dot*vec_to_reachable_set/vec_to_reachable_set_norm
            #     if np.linalg.norm(crosstrack_vec)<distance and 0.<=vec_dot<=vec_to_reachable_set_norm:
            #         distance = np.linalg.norm(crosstrack_vec)
            # else:
                d, proj = distance_point_polytope(p, goal_state,ball='l2')
                if d<distance:
                    projection = proj
                    distance = d
        # if distance<0.5:
        #     # enumerate inputs
        #     potential_inputs = np.linspace(hopper_system.input_limits[0, 0], hopper_system.input_limits[1, 0],
        #                                    input_samples)
        #     for u_i in potential_inputs:
        #         state_list = []
        #         state = reachable_set.parent_state
        #         for step in range(int(step_size / nonlinear_dynamic_step_size)):
        #             state = hopper_system.forward_step(u=np.atleast_1d(u_i), linearlize=False, modify_system=False,
        #                                                  step_size=nonlinear_dynamic_step_size, return_as_env=False,
        #                                                  starting_state=state)
        #             state_list.append(state)
        #             distance = np.linalg.norm(goal_state-state)
        best_distance = min(distance, best_distance)
        if distance < goal_tolerance:
            print('Goal error is %f' % distance)
            return True, [reachable_set.parent_state, goal_state]
        return False, None

    rrt = SymbolicSystem_R3T(hopper_system, gaussian_mixture_sampler, step_size, contains_goal_function=contains_goal_function, use_convex_hull=True, \
                             use_true_reachable_set=True)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('R3T_Hopper_1d_'+experiment_name)
    max_iterations = 100
    for itr in range(max_iterations):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state, stop_on_first_reach=True, allocated_time= 30, rewire=False, explore_deterministic_next_state=True,save_true_dynamics_path =True) is not None:
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
        # print("Best distance %f" %best_distance)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal)
        # fig, ax = visZ(reachable_polytopes, title="", alpha=0.07, fig=fig,  ax=ax, color='gray')
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        # ax.set_aspect('equal')
        plt.plot([l+p, l+p], [-7,7], 'm--', lw=1.5)
        plt.plot([l, l], [-7,7], 'b--', lw=1.5)
        ax.grid(True, which='both')
        # ax.set_xlim(left=0)
        plt.xlim([0.5,4.5])
        plt.ylim([-8,8])
        plt.tight_layout()
        plt.gcf().subplots_adjust(left=0.13, bottom=0.12)
        plt.xlabel('$x(m)$')
        plt.ylabel('$\dot{x}(m/s)$')
        duration += (end_time-start_time)
        plt.title('R3T after %.2f seconds (explored %d nodes)' %(duration, len(polytope_reachable_sets)))
        plt.savefig('R3T_Hopper_1d_'+experiment_name+'/%.2f_seconds.png' % duration, dpi=500)
        # plt.show()
        plt.clf()
        plt.close()

        # # # Plot explored reachable sets
        # # FIXME: Handle degenerated reachable set
        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_2D_AH_polytope(reachable_polytopes, fig=fig, ax=ax,N=200,epsilon=0.01)

        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=5)
        ax.scatter(goal_state[0]-2*np.pi, goal_state[1], facecolor='green', s=5)

        # ax.set_aspect('equal')
        plt.xlabel('$x$')
        plt.ylabel('$\dot{x}$')
        plt.xlim([0.5,4.5])
        plt.ylim([-8,8])
        plt.tight_layout()
        plt.gcf().subplots_adjust(left=0.13, bottom=0.12)
        plt.title('Reachable Set after %.2fs (%d nodes)' %(duration, len(polytope_reachable_sets)))
        plt.savefig('R3T_Hopper_1d_'+experiment_name+'/%.2f_seconds_reachable_sets.png' % duration, dpi=500)
        # plt.show()
        plt.clf()


        if found_goal:
            break


if __name__=='__main__':
    for i in range(10):
        test_hopper_1d_planning()