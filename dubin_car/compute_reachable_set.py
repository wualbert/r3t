import numpy as np
import pickle
from common.rg_rrt_star import *
import os

class DC_Car_Frame_Path(Path):
    def __init__(self):
        '''
        Creates a path object
        FIXME
        '''
        Path.__init__(self)
        pass

    def __repr__(self):
        raise ('NotImplementedError')

    def append(self, path):
        raise ('NotImplementedError')

class Base_DC_Reachable_Set(ReachableSet):
    '''
    A base reachable set for a Dubin's car located at (x,y,theta) = (0,0,0)
    '''
    def __init__(self, x_range=np.asarray([0,10]), y_range=np.asarray([0,10]), x_resolution=0.01,
                 y_resolution=0.01, theta_resolution=0.01,turn_radius = 0.3):
        ReachableSet.__init__(self,DC_Car_Frame_Path)
        self.compute_base_reachable_set()
        self.x_resolution = x_resolution
        self.y_resolution = y_resolution
        self.theta_resolution = theta_resolution
        self.x_count = int((x_range[1]-x_range[0])/self.x_resolution)+1
        self.y_count = int((y_range[1]-y_range[0])/self.y_resolution)+1
        self.theta_count = int(2*np.pi/theta_resolution)
        self.reachabilities = np.zeros([self.x_count,self.y_count,self.theta_count])
        self.turn_radius = turn_radius
        self.compute_base_reachable_set()

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

    def index_to_coordinates(self):

    def coordinates_to_index(self, state):
        x_i = max(int(np.around(state[0]*self.x_resolution)),0)
        y_i = max(int(np.around(state[1] * self.y_resolution)),0)
        theta_i = max(int(np.around(state[1] * self.theta_resolution)),0)

    def compute_dubin_path_to_state(self, goal_state):
        pass

    def compute_base_reachable_set(self):
        for i in range(self.x_count):
            for j in range(self.x_count):
                for k in range(self.theta_count):



if __name__=='__main__':
    base_dc_reachable_set = Base_DC_Reachable_Set()
    dir_path = os.path.dirname(os.path.realpath(__file__))
    with open(dir_path + '/base_reachable_set.p', 'wb+') as f:
        pickle.dump(Base_DC_Reachable_Set,f)