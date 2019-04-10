from common.rg_rrt_star import *
from rtree import index
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque

class DC_Path(Path):
    def __init__(self):
        Path.__init__(self)
        pass

    def __repr__(self):
        raise ('NotImplementedError')

    def append(self, path):
        raise ('NotImplementedError')

class DC_ReachableSet(ReachableSet):
    '''
    Base class of ReachableSet
    '''
    def __init__(self,path_class):#, state, planner, in_set, collision_test):
        '''

        :param state: The state this reachable set belongs to
        :param planner: Local planner for planning paths between self.state and any point in the reachable set.
        :param in_set: A fast checker for checking whether a state is in the set
        Planner takes in (state, goal) and returns (cost_to_go, path)
        '''
        ReachableSet.__init__(self,path_class)
        pass
        # self.state = state
        # self.state_dim = state.shape[0]
        # self.planner = planner
        # self.in_set = in_set
        # self.collision_test = collision_test

    def contains(self, goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        raise('NotImplementedError')

    def plan_collision_free_path_in_set(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        # if not self.contains(goal_state):
        #     return (np.inf, None)
        # return self.planner(self.state, goal_state)
        raise('NotImplementedError')


    def find_closest_state(self, query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        raise('NotImplementedError')

class DC_Map:
    def __init__(self):
        pass

class DC_RGRRTStar(RGRRTStar):
    def __init__(self, root_state, compute_reachable_set, sampler, state_tree, path_class,rewire_radius = None):
        RGRRTStar.__init__(self)
        pass
    def visualize(self):
        pass