import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pydrake.symbolic as sym
from timeit import default_timer
from r3t.symbolic_system.symbolic_system_r3t import *
from r3t.symbolic_system.examples.hopper_2D_visualize import hopper_plot
from r3t.utils.visualization import visualize_node_tree_hopper_2D, visualize_node_tree_2D
from polytope_symbolic_system.examples.hopper_2d import Hopper_2d
from pypolycontain.lib.polytope import polytope
from pypolycontain.visualization.visualize_2D import visualize_2D_zonotopes as visZ
from pypolycontain.lib.operations import distance_point_polytope, distance_polytopes
from collections import deque
import time
from datetime import datetime
import os
import pickle
matplotlib.rcParams['font.family'] = "Times New Roman"
matplotlib.rcParams.update({'font.size': 14})

class Hopper2D_ReachableSet(PolytopeReachableSet):
    def __init__(self, parent_state, polytope_list, sys, epsilon=1e-3, contains_goal_function = None, deterministic_next_state = None, ground_height_function = lambda x:0, reachable_set_step_size=1e-1, nonlinear_dynamic_step_size=1e-3):
        PolytopeReachableSet.__init__(self, parent_state, polytope_list, sys=sys, epsilon=epsilon, contains_goal_function=contains_goal_function, \
                                      use_true_reachable_set=True, reachable_set_step_size=reachable_set_step_size, nonlinear_dynamic_step_size=nonlinear_dynamic_step_size, deterministic_next_state=deterministic_next_state)
        self.ground_height_function = ground_height_function
        self.body_attitude_limit = np.pi/2-1e-2
        self.leg_attitude_limit = np.pi/3
        self.ground_height_function=ground_height_function
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
        if goal_state[4]+goal_state[9]*2e-2<0.2 or goal_state[4]+goal_state[9]*2e-2>8:
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

class Hopper2D_RGRRTStar(SymbolicSystem_R3T):
    def __init__(self, sys, sampler, step_size, nonlinear_dynamic_step_size, contains_goal_function = None, ground_height_function=lambda x:0):
        self.sys = sys
        self.step_size = step_size
        self.nonlinear_dynamic_step_size = nonlinear_dynamic_step_size
        self.contains_goal_function = contains_goal_function
        self.ground_height_function=ground_height_function
        def compute_reachable_set(state):
            '''
            Compute zonotopic reachable set using the system
            :param h:
            :return:a
            '''
            deterministic_next_state = None
            reachable_set_polytope = self.sys.get_reachable_polytopes(state)
            # if state[1] <= 0:
            #     print('state', state)
            #     print("T", reachable_set_polytope[0].T)
            #     print("t", reachable_set_polytope[0].t)
            #     print("H", reachable_set_polytope[0].P.H)
            #     print("h", reachable_set_polytope[0].P.h)
            #TODO: collision check here
            if np.all(self.sys.get_linearization(state=state).B == 0):
                deterministic_next_state = [state]
                for step in range(int(self.step_size / self.nonlinear_dynamic_step_size)):
                    state = self.sys.forward_step(starting_state=state, modify_system=False, return_as_env=False,
                                                  step_size=nonlinear_dynamic_step_size)
                    deterministic_next_state.append(state)
            return Hopper2D_ReachableSet(state,reachable_set_polytope, sys=self.sys, contains_goal_function=self.contains_goal_function,\
                                         deterministic_next_state=deterministic_next_state, reachable_set_step_size=self.step_size,\
                                         nonlinear_dynamic_step_size=self.nonlinear_dynamic_step_size, ground_height_function=self.ground_height_function)
        def convert_to_cg_state(foot_state):
            cg_state = np.zeros(0)
            cg_state[0]

        SymbolicSystem_R3T.__init__(self, sys, sampler, step_size, contains_goal_function, compute_reachable_set)

def test_hopper_2d_planning(initial_state = np.asarray([0., 1., 0, 0, 1.5, 0., 0., 0., 0., 0.]), goal_state = np.asarray([10.,1.,0.,0.,1.5,0.,0.,0.,0.,0.]),\
                            ground_height_function = lambda x:0, save_animation=False):
    hopper_system = Hopper_2d(initial_state=initial_state,ground_height_function=ground_height_function)
    # [theta1, theta2, x0, y0, w]
    # from x0 = 0 move to x0 = 5
    goal_tolerance = np.hstack([[1], np.ones(9)*100])
    step_size = 1e-1
    nonlinear_dynamic_step_size=5e-3
    #TODO
    def uniform_sampler():
        rnd = np.random.rand(10)
        rnd[0] = rnd[0]*10-0.5
        rnd[1] = (rnd[1]-0.5)*2+3
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
        rnd[2] = np.random.normal(0, np.pi/4) #np.random.normal(0, np.pi/6)
        rnd[3] = np.random.normal(0, np.pi/6) #np.random.normal(0, np.pi/16)
        rnd[4] = (rnd[4]-0.5)*2*4+5
        rnd[5] = (rnd[5]-0.5)*2*3
        rnd[6] = (rnd[5]-0.5)*2*6#np.random.normal(0, 6)
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
        rnd[0] = rnd[0] * 20 - 3
        rnd[1] = (rnd[1] - 0.5) * 2 * 0.75 + 1.5 + ground_height_function(rnd[0])
        rnd[2] = np.random.normal(0, np.pi / 4) # (np.random.rand(1)-0.5)*2*np.pi/12
        rnd[3] = np.random.normal(0, np.pi / 8)#np.random.normal(0, np.pi / 16)
        rnd[4] = (rnd[4] - 0.5) * 2 * 0.5 + 4
        rnd[5] = np.random.normal(1.5, 3) #(rnd[5] - 0.5) * 2 * 6
        rnd[6] = (rnd[6] - 0.5) * 2 * 12 # np.random.normal(0, 6)
        rnd[7] = np.random.normal(0, 20) # (np.random.rand(1)-0.5)*2*20
        rnd[8] = np.random.normal(0, 3) # (np.random.rand(1)-0.5)*2*5
        rnd[9] = (rnd[9] - 0.5) * 2 * 10 + 3 #np.random.normal(2, 12)
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
        if np.random.rand(1)<0.5:
            return hip_coordinates_sampler()
        else:
            return contact_sampler()

    def contains_goal_function(reachable_set, goal_state):
        # distance = np.inf
        # projection = None
        # for p in reachable_set.polytope_list:
        #     d, proj = distance_point_polytope(p, goal_state)
        #     # if d<distance:
        #     #     projection = np.asarray(proj)
        #     #     distance = d
        # print((projection-goal_state))
        x_ft = reachable_set.parent_state[0]
        theta = reachable_set.parent_state[2]
        r = reachable_set.parent_state[4]
        x_body = x_ft+r*np.sin(theta)
        if (x_body-np.ndarray.flatten(goal_state)[0])>0:
            return True, [reachable_set.deterministic_next_state]
        return False, None

    rrt = Hopper2D_RGRRTStar(hopper_system, hip_coordinates_sampler, step_size, nonlinear_dynamic_step_size=nonlinear_dynamic_step_size, contains_goal_function=contains_goal_function)
    found_goal = False
    experiment_name = datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H-%M-%S')

    duration = 0
    os.makedirs('RRT_Hopper_2d_'+experiment_name)
    max_iterations = 10
    allocated_time = 5

    for itr in range(max_iterations):
        start_time = time.time()
        if rrt.build_tree_to_goal_state(goal_state, stop_on_first_reach=False, allocated_time= allocated_time, rewire=True, explore_deterministic_next_state=True,save_true_dynamics_path = True) is not None:
            found_goal = True
        end_time = time.time()
        #get rrt polytopes
        polytope_reachable_sets = rrt.reachable_set_tree.id_to_reachable_sets.values()
        reachable_polytopes = []
        explored_states = []
        for prs in polytope_reachable_sets:
            reachable_polytopes.extend(prs.polytope_list)
            explored_states.append(prs.parent_state)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_hopper_2D(rrt, fig, ax, s=0.5, linewidths=0.15, show_path_to_goal=found_goal, dims=[0,1], ground_height_function=ground_height_function)
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=5)
        # ax.set_aspect('equal')
        # plt.plot([l+p, l+p], [-2.5, 2.5], 'm--', lw=1.5)
        #plot goal
        current_ylim = ax.get_ylim()
        plt.plot([goal_state[0],goal_state[0]], [current_ylim[0]-1, 6], 'g--', lw=3, alpha=0.8)
        # plot ground
        current_xlim = ax.get_xlim()
        x_samples = np.linspace(ax.get_xlim()[0]-1, ax.get_xlim()[1]+1, num=100)
        vec_ground = np.vectorize(ground_height_function)
        ax.plot(x_samples, vec_ground(x_samples),'-',linewidth=3,markersize=10, color='saddlebrown', alpha=0.5)

        ax.set_xlim(current_xlim)
        ax.set_ylim(bottom=current_ylim[0],top=5)
        plt.xlabel('$x$(m)')
        plt.ylabel('$y$(m)')
        duration += (end_time-start_time)
        plt.title('R3T after %.2f seconds (explored %d nodes)' %(duration, rrt.node_tally),pad=15)
        plt.tight_layout()
        plt.savefig('RRT_Hopper_2d_'+experiment_name+'/%.2f_seconds.png' % duration, dpi=500)

        # plt.show()
        plt.clf()
        plt.close()

        # store polytopes
        reachable_polytopes_clean = [[p.T, p.t, p.P.H, p.P.h] for p in reachable_polytopes]
        with open('RRT_Hopper_2d_'+experiment_name+'/%.2f_seconds_reachable_sets.p' % duration, "wb") as f:
            pickle.dump(reachable_polytopes_clean, f)
        allocated_time*=2

def staircase_ground_height_function(x):
    if isinstance(x, sym.Variable):
        e3 = -sym.exp(10 * (x - 3))
        e7 = -sym.exp(10 * (x - 7))
        return 0.5 / (1 + e3) - 0.5 / (1 + e7)
    if 3<x<7:
        return 0.5
    else:
        return 0


def ramp_ground_height_function(x):
    return 0.05*x

if __name__=='__main__':
    for i in range(1):
        test_hopper_2d_planning(initial_state=np.asarray([0., 1., 0, 0, 1.5, 0., 0., 0., 0., 0.]),
                                    goal_state=np.asarray([10., 1., 0., 0., 1.5, 0., 0., 0., 0., 0.]), \
                                    save_animation=False)
