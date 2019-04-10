import numpy as np
import pickle
from common.rg_rrt_star import *
from dubin_car_rg_rrt_star import *

class Base_DC_Reachable_Set(ReachableSet):
    '''
    A base reachable set for a Dubin's car located at (x,y,theta) = (0,0,0)
    '''
    def __init__(self, x_range, y_range, theta_range, x_resolution, y_resolution, theta_resolution):
        ReachableSet.__init__(self,DC_Path)


    def contains(self, goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        raise ('NotImplementedError')

    def plan_collision_free_path_in_set(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        # if not self.contains(goal_state):
        #     return (np.inf, None)
        # return self.planner(self.state, goal_state)
        raise ('NotImplementedError')

    def find_closest_state(self, query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        raise ('NotImplementedError')

