import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer
from rg_rrt_star.symbolic_system.symbolic_system_rg_rrt_star import *
from rg_rrt_star.symbolic_system.examples.hopper_2D_visualize import hopper_plot
from rg_rrt_star.utils.visualization import visualize_node_tree_hopper_2D, visualize_node_tree_2D
from polytope_symbolic_system.examples.hopper_2d import Hopper_2d
from pypolycontain.lib.polytope import polytope
from pypolycontain.visualization.visualize_2D import visualize_2D_zonotopes as visZ
from pypolycontain.lib.operations import distance_point_polytope, distance_polytopes
from collections import deque
import time
from datetime import datetime
import os

class Hopper2D_ReachableSet(PolytopeReachableSet):
    def __init__(self, parent_state, polytope_list, epsilon=1e-3, contains_goal_function = None, deterministic_next_state = None, ground_height_function = lambda x:0):
        PolytopeReachableSet.__init__(self, parent_state, polytope_list, epsilon, contains_goal_function, deterministic_next_state)
        self.ground_height_function = ground_height_function
        self.body_attitude_limit = np.pi/4
        self.leg_attitude_limit = np.pi/3
    def plan_collision_free_path_in_set(self, goal_state, return_deterministic_next_state = False):
        #fixme: support collision checking
        #check for impossible configurations

        # tipped over
        if goal_state[2]+goal_state[7]*2e-2>self.leg_attitude_limit or goal_state[2]+goal_state[7]*2e-2<-self.leg_attitude_limit:
            # print('leg tipped over')
            if return_deterministic_next_state:
                return np.inf, None, None
            else:
                return np.inf, None

        # # body attitude is off
        # if goal_state[3]+goal_state[8]*2e-2>self.body_attitude_limit or goal_state[3]+goal_state[8]*2e-2<-self.body_attitude_limit:
        #     # print('body attitude off')
        #     if return_deterministic_next_state:
        #         return np.inf, None, None
        #     else:
        #         return np.inf, None

        # stuck in the ground
        if goal_state[1]<self.ground_height_function(goal_state[0])-1 or goal_state[1]>self.ground_height_function(goal_state[0])+8:
            # print('invalid height')
            if return_deterministic_next_state:
                return np.inf, None, None
            else:
                return np.inf, None

        # stuck in the ground
        if goal_state[4]+goal_state[9]*2e-2<0.2:
            # print('r invalid')
            if return_deterministic_next_state:
                return np.inf, None, None
            else:
                return np.inf, None
        #
        # is_contain, closest_state = self.contains(goal_state)
        # if not is_contain:
        #     print('Warning: this should never happen')
        #     return np.linalg.norm(self.parent_state-closest_state), deque([self.parent_state, closest_state]) #FIXME: distance function

        # Simulate forward dynamics if there can only be one state in the next timestep
        if not return_deterministic_next_state:
            return np.linalg.norm(self.parent_state-goal_state), deque([self.parent_state, goal_state])
        return np.linalg.norm(self.parent_state-goal_state), deque([self.parent_state, goal_state]), self.deterministic_next_state

    # def find_closest_state(self, query_point):
    #     '''
    #     Find the closest state from the query point to a given polytope
    #     :param query_point:
    #     :return: Tuple (closest_point, closest_point_is_self.state)
    #     '''
    #     distance = np.inf
    #     closest_point = None
    #     try:
    #         #use AABB to upper bound distance
    #         min_dmax = np.inf
    #         for i, aabb in enumerate(self.aabb_list):
    #             dmax = point_to_box_dmax(query_point, aabb)
    #             if dmax < min_dmax:
    #                 min_dmax = dmax
    #
    #         for i, p in enumerate(self.polytope_list):
    #             #ignore polytopes that are impossible
    #             if point_to_box_distance(query_point, self.aabb_list[i]) > min_dmax:
    #                 continue
    #             d, proj = distance_point(p, query_point)
    #             if d<distance:
    #                 distance = d
    #                 closest_point = proj
    #         assert closest_point is not None
    #     except TypeError:
    #         closest_point = distance_point(self.polytope_list, query_point)[1]
    #     closest_point = np.ndarray.flatten(closest_point)
    #     return closest_point, np.linalg.norm(closest_point-self.parent_state)<self.epsilon

class Hopper2D_RGRRTStar(SymbolicSystem_RGRRTStar):
    def __init__(self, sys, sampler, step_size, contains_goal_function = None):
        self.sys = sys
        self.step_size = step_size
        self.contains_goal_function = contains_goal_function
        def compute_reachable_set(state):
            '''
            Compute zonotopic reachable set using the system
            :param h:
            :return:a
            '''
            deterministic_next_state = None
            reachable_set_polytope = self.sys.get_reachable_polytopes(state, step_size=self.step_size)
            # if state[1] <= 0:
            #     print('state', state)
            #     print("T", reachable_set_polytope[0].T)
            #     print("t", reachable_set_polytope[0].t)
            #     print("H", reachable_set_polytope[0].P.H)
            #     print("h", reachable_set_polytope[0].P.h)
            #TODO: collision check here
            if np.all(self.sys.get_linearization(state=state).B == 0):
                deterministic_next_state = self.sys.forward_step(starting_state=state, modify_system=False, return_as_env=False, step_size=self.step_size)
            return Hopper2D_ReachableSet(state,reachable_set_polytope, contains_goal_function=self.contains_goal_function, deterministic_next_state=deterministic_next_state)
        SymbolicSystem_RGRRTStar.__init__(self, sys, sampler, step_size, contains_goal_function, compute_reachable_set)

def test_hopper_2d_planning():
    initial_state = np.asarray([0., 0.5, -0.1, 0, 4, 0., 0., 0., 0., 0.])
    hopper_system = Hopper_2d(initial_state=initial_state)
    # [theta1, theta2, x0, y0, w]
    # from x0 = 0 move to x0 = 5
    goal_state = np.asarray([5.,0.,0.,0.,5.,0.,0.,0.,0.,0.])
    goal_tolerance = np.hstack([[1], np.ones(9)*100])
    step_size = 1e-1
    #TODO
    def uniform_sampler():
        rnd = np.random.rand(10)
        rnd[0] = rnd[0]*10-0.5
        rnd[1] = (rnd[1]-0.5)*10+3
        rnd[2] = np.random.normal(0, np.pi/12)
        rnd[3] = np.random.normal(0, np.pi/16)
        rnd[4] = (rnd[4]-0.5)*2*4+5
        rnd[5] = (rnd[5]-0.5)*2*3
        rnd[6] = (rnd[5]-0.5)*2*10 #np.random.normal(0, 6)
        rnd[7] = np.random.normal(0, 2)
        rnd[8] = np.random.normal(0, 2)
        rnd[9] = (rnd[9] - 0.1) * 2 * 20
        # goal_bias = np.random.rand(1)
        return rnd
    def contact_sampler():
        rnd = np.random.rand(10)
        rnd[0] = rnd[0]*10-0.5
        rnd[1] = np.random.normal(-0.5,0.2)
        rnd[2] = np.random.normal(0, np.pi/6)
        rnd[3] = np.random.normal(0, np.pi/16)
        rnd[4] = (rnd[4]-0.5)*2*4+5
        rnd[5] = (rnd[5]-0.5)*2*3
        rnd[6] = (rnd[5]-0.5)*2*8 + 3#np.random.normal(0, 6)
        rnd[7] = np.random.normal(0, 30)
        rnd[8] = np.random.normal(0, 2)
        rnd[9] = (rnd[9] - 0.1) * 2 * 20
        # goal_bias = np.random.rand(1)
        return rnd
    def hip_coordinates_sampler():
        # [x_hip, y_hip, theta(leg), phi(body), r]
        # if np.random.rand(1)<0.5:
        #     return uniform_sampler()
        rnd = np.random.rand(10)
        rnd[0] = rnd[0] * 10 - 3
        rnd[1] = (rnd[1] - 0.5) * 2 * 2.5 + 4
        rnd[2] = np.random.normal(0, np.pi / 12) # (np.random.rand(1)-0.5)*2*np.pi/12
        rnd[3] = np.random.normal(0, np.pi / 10)#np.random.normal(0, np.pi / 16)
        rnd[4] = (rnd[4] - 0.5) * 2 * 2 + 4
        rnd[5] = np.random.normal(1.5, 1.5) #(rnd[5] - 0.5) * 2 * 6
        rnd[6] = (rnd[6] - 0.5) * 2 * 10 # np.random.normal(0, 6)
        rnd[7] = np.random.normal(0, 3) # (np.random.rand(1)-0.5)*2*20
        rnd[8] = np.random.normal(0, 0.05) # (np.random.rand(1)-0.5)*2*5
        rnd[9] = (rnd[9] - 0.5) * 2 * 5 + 3 #np.random.normal(2, 12)
        # convert to hopper foot coordinates
        rnd_ft = np.zeros(10)
        rnd_ft[0] = rnd[0]-np.sin(rnd[2])*rnd[4]
        rnd_ft[1] = rnd[0]-np.cos(rnd[2])*rnd[4]
        # if np.random.rand(1)<0.7:
        #     rnd_ft[1]=(rnd[1]/2-0.2)

        rnd_ft[5] = rnd[5]-rnd[9]*np.sin(rnd[2])-rnd[4]*np.cos(rnd[2])*rnd[7]
        rnd_ft[6] = rnd[6] - rnd[9] * np.cos(rnd[2]) + rnd[4] * np.sin(rnd[2]) * rnd[7]
        if rnd_ft[1]<=0:
            rnd_ft[2]=rnd[2]*2
            rnd_ft[7]=rnd[7]*5
        else:
            rnd_ft[2] = rnd[2]
            rnd_ft[7] = rnd[7]
        rnd_ft[3:5] = rnd[3:5]
        rnd_ft[8:] = rnd[8:]
        return rnd_ft

    def hybrid_sampler():
        # samples contact and flight separately
        raise NotImplementedError

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
        # print((projection-goal_state))
        if np.all(abs(projection-goal_state)<goal_tolerance):
            return True
        return False

    rrt = Hopper2D_RGRRTStar(hopper_system, hip_coordinates_sampler, step_size, contains_goal_function=contains_goal_function)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('RRT_Hopper_2d_'+experiment_name)
    max_iterations = 10000
    for itr in range(max_iterations):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state, stop_on_first_reach=True, allocated_time= 15, rewire=True, explore_deterministic_next_state=True) is not None:
            found_goal = True
        end_time = time.time()
        #get rrt polytopes
        # polytope_reachable_sets = rrt.reachable_set_tree.id_to_reachable_sets.values()
        # reachable_polytopes = []
        # explored_states = []
        # for prs in polytope_reachable_sets:
        #     reachable_polytopes.extend(prs.polytope_list)
        #     explored_states.append(prs.parent_state)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_hopper_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal, dims=[0,1])
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        # ax.set_aspect('equal')
        # plt.plot([l+p, l+p], [-2.5, 2.5], 'm--', lw=1.5)
        plt.plot([5,5], [-2.5, 2.5], 'g--', lw=1.5)

        # ax.set_xlim(left=0)
        plt.xlabel('$x_0$')
        plt.ylabel('$y_0$')
        duration += (end_time-start_time)
        plt.title('RRT Tree after %.2f seconds (explored %d nodes)' %(duration, rrt.node_tally))
        plt.savefig('RRT_Hopper_2d_'+experiment_name+'/%.2f_seconds.png' % duration, dpi=500)

        # plt.show()
        plt.clf()
        plt.close()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal, dims=[1,4])
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[1], initial_state[4], facecolor='red', s=5)

        # ax.set_xlim(left=0)
        plt.xlabel('$y_0$')
        plt.ylabel('$r$')
        plt.title('RRT Tree after %.2f seconds (explored %d nodes)' %(duration, rrt.node_tally))
        plt.savefig('RRT_Hopper_2d_'+experiment_name+'/%.2f_seconds_2.png' % duration, dpi=500)

        # plt.show()
        plt.clf()
        plt.close()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal, dims=[0,4])
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[4], facecolor='red', s=5)

        # ax.set_xlim(left=0)
        plt.xlabel('$x_0$')
        plt.ylabel('$r$')
        plt.title('RRT Tree after %.2f seconds (explored %d nodes)' %(duration, rrt.node_tally))
        plt.savefig('RRT_Hopper_2d_'+experiment_name+'/%.2f_seconds_3.png' % duration, dpi=500)
        plt.clf()
        plt.close()

        # # save hopper animation
        # os.makedirs('RRT_Hopper_2d_' + experiment_name + '/animation_%.2f_seconds' % duration)
        # # find state closest to goal
        # nearest_box = np.hstack([goal_state[0:5], np.ones(5)*-1000, goal_state[0:5], np.ones(5)*1000])
        # closest_state_to_goal = list(rrt.state_tree.state_idx.nearest(nearest_box,1))[0]
        # node_to_plot = rrt.state_to_node_map[closest_state_to_goal]
        # states_to_plot = []
        # while(1):
        #     states_to_plot.append(node_to_plot.state)
        #     if node_to_plot.parent is None:
        #         break
        #     else:
        #         node_to_plot = node_to_plot.parent
        # states_to_plot.reverse()
        # for index, s in enumerate(states_to_plot):
        #     fig = plt.figure()
        #     ax = fig.add_subplot(111)
        #     hopper_plot(s, ax, fig)
        #     plt.savefig('RRT_Hopper_2d_' + experiment_name + '/animation_%.2f_seconds/%i' % (duration, index), dpi=500)
        #     plt.clf()
        #     plt.close()

        # plt.show()
        if found_goal:
            break


if __name__=='__main__':
    test_hopper_2d_planning()