import numpy as np
import pickle
from common.rg_rrt_star import *
import os
from collections import deque
from utils.utils import *
from bounding_box_closest_polytope.lib.box import AABB, point_in_box
import dubins

class DC_Car_Frame_Path(Path):
    '''
    Wrapper for dubins path object
    '''
    def __init__(self, end):
        '''
        Creates a path object
        FIXME: This implementation is kinda stupid
        '''
        Path.__init__(self)
        self.end = end

    def __repr__(self):
        return('Car frame Dubin path '+ str(self.end))

    def get_fraction_state(self, fraction):
        raise(NotImplementedError)

class Base_DC_Reachable_Set(ReachableSet):
    '''
    A base reachable set for a Dubin's car located at (x,y,theta) = (0,0,0)
    '''
    def __init__(self, x_range=np.asarray([0,5]), y_range=np.asarray([-5,5]), x_resolution=0.1,
                 y_resolution=0.1, theta_resolution=0.3,turn_radius = 1):
        ReachableSet.__init__(self,DC_Car_Frame_Path)
        self.x_range = x_range
        self.y_range = y_range
        self.x_resolution = x_resolution
        self.y_resolution = y_resolution
        self.theta_resolution = theta_resolution
        self.x_count = int(np.ceil((self.x_range[1]-self.x_range[0])/self.x_resolution))
        self.y_count = int(np.ceil((self.y_range[1]-self.y_range[0])/self.y_resolution))
        self.theta_count = int(np.ceil(2*np.pi/theta_resolution))
        self.reachabilities = np.empty([self.x_count,self.y_count,self.theta_count],dtype=bool)
        self.paths = np.empty([self.x_count,self.y_count,self.theta_count],dtype=object)
        self.costs = np.empty([self.x_count,self.y_count,self.theta_count],dtype=float)
        self.turn_radius = turn_radius
        self.compute_base_reachable_set()
        self.AABB = AABB(([self.x_range[0],self.y_range[0]],[self.x_range[1],self.y_range[1]]))

    def contains(self, goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        #check if the point is within x and y limits
        #notice theta is all covered, so no need to check
        if point_in_box(self.AABB,goal_state):
            return False
        #actually try to query
        x_index, y_index, theta_index = self.coordinates_to_index(goal_state)
        return self.reachabilities[x_index,y_index,theta_index]

    def plan_collision_free_path_in_set(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        # if not self.contains(goal_state):
        #     return (np.inf, None)
        # return self.planner(self.state, goal_state)
        assert(self.contains(goal_state))
        x_index, y_index, theta_index = self.coordinates_to_index(goal_state)
        return self.costs[x_index,y_index,theta_index], self.paths[x_index,y_index,theta_index]

    def find_closest_state(self, query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_state, closest_point_is_self.state)
        '''
        closest_state = np.zeros(3)
        closest_state[0] = min(max(query_point[0],self.x_range[0]),self.x_range[1])
        closest_state[1] = min(max(query_point[1],self.y_range[0]),self.y_range[1])
        closest_state[2] = wrap_angle(query_point[2])
        return closest_state

    def index_to_coordinates(self, i,j,k):
        assert(0<= i < self.x_count and 0 <= j <self.y_count and 0 <= k <self.theta_count)
        x = self.x_range[0]+i*self.x_resolution
        y = self.y_range[0]+j*self.y_resolution
        theta = wrap_angle(k*self.theta_resolution-np.pi)
        return np.asarray([x,y,theta])

    def coordinates_to_index(self, state):
        x_i = max(int(np.around(state[0] / self.x_resolution)),0)
        y_i = max(int(np.around(state[1] / self.y_resolution)),0)
        theta_i = max(int(np.around((state[2]-np.pi) / self.theta_resolution)),0)
        return np.asarray([x_i,y_i,theta_i])

    def compute_dubin_path_to_state(self, goal_state):
        #compute dubin's path
        path = dubins.shortest_path(np.zeros(3), goal_state, self.turn_radius)
        cost = path.path_length()
        #check for undesirable paths: ones that requires turning over pi/2
        step_size = cost/20 #FIXME: arbitrary number
        samples = path.sample_many(step_size)[0]
        for s in samples:
            if abs(wrap_angle(s[2]))>np.pi/2:
                return False, None, np.inf
        return True, DC_Car_Frame_Path(goal_state), cost

    def compute_base_reachable_set(self):
        print('Computing base reachable set...')
        for i in range(self.x_count):
            for j in range(self.y_count):
                for k in range(self.theta_count):
                    self.reachabilities[i,j,k], self.paths[i,j,k],self.costs[i,j,k] = self.compute_dubin_path_to_state(self.index_to_coordinates(i,j,k))

if __name__=='__main__':
    base_dc_reachable_set = Base_DC_Reachable_Set()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    with open(dir_path + '/base_reachable_set.p', 'wb+') as f:
        pickle.dump(base_dc_reachable_set,f)