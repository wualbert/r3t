#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 16:51:26 2019

@author: sadra
"""

import pydrake
from r3t.common.r3t import *
from r3t.symbolic_system.symbolic_system_r3t import PolytopePath
from polytope_symbolic_system.common.symbolic_system import *
from pypolycontain.lib.operations import distance_point_polytope
from collections import deque
from rtree import index
from closest_polytope_algorithms.bounding_box.polytope_tree import PolytopeTree
from closest_polytope_algorithms.bounding_box.box import AH_polytope_to_box, \
    point_to_box_dmax, point_to_box_distance
    
class PolytopeReachableSet(ReachableSet):
    def __init__(self, parent_state, polytope_list, sys, epsilon=1e-3, contains_goal_function = None, deterministic_next_state = None, \
                 use_true_reachable_set=False, reachable_set_step_size=None, nonlinear_dynamic_step_size=1e-2):
        ReachableSet.__init__(self, parent_state=parent_state, path_class=PolytopePath)
        self.polytope_list = polytope_list
        try:
            self.aabb_list = [AH_polytope_to_box(p, return_AABB=True) for p in self.polytope_list]
        except TypeError:
            self.aabb_list = None
        self.epsilon = epsilon
        self.deterministic_next_state = deterministic_next_state
        self.use_true_reachable_set = use_true_reachable_set
        self.reachable_set_step_size = reachable_set_step_size
        self.nonlinear_dynamic_step_size = nonlinear_dynamic_step_size
        # try:
        #     self.parent_distance = min([distance_point(p, self.parent_state)[0] for p in self.polytope_list])
        # except TypeError:
        #     self.parent_distance = distance_point(self.polytope_list, self.parent_state)[0]

        self.contains_goal_function = contains_goal_function
        # assert(self.parent_distance<self.epsilon)

    def contains(self, goal_state, return_closest_state = True):
        # print(distance_point(self.polytope, goal_state)[0])
        # print(self.polytope)
        try:
            #multimodal
            distance = np.inf
            closest_state = None
            for i, polytope in enumerate(self.polytope_list):
                # if point_to_box_distance(goal_state, self.aabb_list[i])>0:
                #     continue
                current_distance, current_closest_state = distance_point_polytope(polytope, goal_state, ball='l2')
                if current_distance < self.epsilon:
                    if return_closest_state:
                        return True, goal_state
                    else:
                        return True
                else:
                    if current_distance < distance:
                        distance = current_distance
                        closest_state = current_closest_state
            return False, closest_state
        except TypeError:
            distance, closest_state = distance_point_polytope(self.polytope_list, goal_state, ball='l2')
            if distance < self.epsilon:
                if return_closest_state:
                    return True, closest_state
                else:
                    return True
            if return_closest_state:
                return False, closest_state
            return False

    def contains_goal(self, goal_state):
        # check if goal is epsilon away from the reachable sets
        if self.contains_goal_function:
            return self.contains_goal_function(self, goal_state)
        raise NotImplementedError


    def find_closest_state(self, query_point, save_true_dynamics_path=False):
        '''
        Find the closest state from the query point to a given polytope
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        distance = np.inf
        closest_point = None
        p_used = None
        try:
            #use AABB to upper bound distance
            min_dmax = np.inf
            for i, aabb in enumerate(self.aabb_list):
                dmax = point_to_box_dmax(query_point, aabb)
                if dmax < min_dmax:
                    min_dmax = dmax
            for i, p in enumerate(self.polytope_list):
                #ignore polytopes that are impossible
                if point_to_box_distance(query_point, self.aabb_list[i]) > min_dmax:
                    continue
                d, proj = distance_point_polytope(p, query_point, ball='l2')
                if d<distance:
                    distance = d
                    closest_point = proj
                    p_used = p
            assert closest_point is not None
        except TypeError:
            closest_point = distance_point_polytope(self.polytope_list, query_point, ball='l2')[1]
            p_used = self.polytope_list
        if np.linalg.norm(closest_point-self.parent_state)<self.epsilon:
            if save_true_dynamics_path:
                return np.ndarray.flatten(closest_point), True, np.asarray([])
            return np.ndarray.flatten(closest_point), True, np.asarray([self.parent_state, np.ndarray.flatten(closest_point)])
        return np.ndarray.flatten(closest_point), False, np.asarray([self.parent_state, np.ndarray.flatten(closest_point)])

    def plan_collision_free_path_in_set(self, goal_state, return_deterministic_next_state = False):
        try:
            if self.plan_collision_free_path_in_set_function:
                return self.plan_collision_free_path_in_set_function(goal_state, return_deterministic_next_state)
        except AttributeError:
            pass
        #fixme: support collision checking
        #
        # is_contain, closest_state = self.contains(goal_state)
        # if not is_contain:
        #     print('Warning: this should never happen')
        #     return np.linalg.norm(self.parent_state-closest_state), deque([self.parent_state, closest_state]) #FIXME: distance function

        # Simulate forward dynamics if there can only be one state in the next timestep
        if not return_deterministic_next_state:
            return np.linalg.norm(self.parent_state-goal_state), deque([self.parent_state, goal_state])
        return np.linalg.norm(self.parent_state-goal_state), deque([self.parent_state, goal_state]), self.deterministic_next_state

class SymbolicSystem_R3T_Oracle(R3T):
    def __init__(self, sys, sampler, step_size, contains_goal_function = None, compute_reachable_set=None, use_true_reachable_set=False, \
                 nonlinear_dynamic_step_size=1e-2, use_convex_hull=True, goal_tolerance = 1e-2):
        self.step_size = step_size
        self.contains_goal_function = contains_goal_function
        self.goal_tolerance = goal_tolerance
        if compute_reachable_set is None:
            def compute_reachable_set(state):
                '''
                Compute polytopic reachable set using the system
                :param h:
                :return:
                '''
                list_of_reachable_sets=oracle(mysystem,x)
                    
                        deterministic_next_state = [state, self.sys.forward_step(starting_state=state, modify_system=False, return_as_env=False, step_size=self.step_size)]
                return PolytopeReachableSet(state,list_of_reachable_sets, contains_goal_function=self.contains_goal_function, \
                                            deterministic_next_state=deterministic_next_state, reachable_set_step_size=self.step_size, use_true_reachable_set=use_true_reachable_set,\
                                            nonlinear_dynamic_step_size=nonlinear_dynamic_step_size)
        RGRRTStar.__init__(self, self.sys.get_current_state(), compute_reachable_set, sampler, PolytopeReachableSetTree, SymbolicSystem_StateTree, PolytopePath)