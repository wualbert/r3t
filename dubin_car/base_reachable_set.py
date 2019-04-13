import numpy as np
# import cPickle as pickle
# import dill
from common.rg_rrt_star import *
import os
from collections import deque
from utils.utils import *
from bounding_box_closest_polytope.lib.box import AABB, point_in_box
import dubins
from time import clock

class Base_DC_Reachable_Set(ReachableSet):
    '''
    A base reachable set for a Dubin's car located at (x,y,theta) = (0,0,0)
    '''
    def __init__(self, x_range=np.asarray([0,10]), y_range=np.asarray([-5,5]), x_resolution=0.05,
                 y_resolution=0.05, theta_resolution=0.01,turn_radius = 0.5, is_reachables= None, costs = None):
        ReachableSet.__init__(self)
        self.x_range = x_range
        self.y_range = y_range
        self.x_resolution = x_resolution
        self.y_resolution = y_resolution
        self.theta_resolution = theta_resolution
        self.x_count = int(np.ceil((self.x_range[1]-self.x_range[0])/self.x_resolution))
        self.y_count = int(np.ceil((self.y_range[1]-self.y_range[0])/self.y_resolution))
        self.theta_count = int(np.ceil(2*np.pi/theta_resolution))
        self.turn_radius = turn_radius
        self.AABB = AABB(([self.x_range[0],self.y_range[0]],[self.x_range[1],self.y_range[1]]))
        self.origin_index = self.coordinates_to_index(np.zeros(3))
        if is_reachables is None or costs is None:
            self.is_reachables = np.empty([self.x_count, self.y_count, self.theta_count], dtype=bool)
            self.costs = np.empty([self.x_count,self.y_count,self.theta_count],dtype=float)
            self.compute_base_reachable_set()
        else:
            self.is_reachables=is_reachables
            self.costs=costs

    def contains(self, car_frame_goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        #check if the point is within x and y limits
        #notice theta is all covered, so no need to check
        if not point_in_box(car_frame_goal_state[0:2], self.AABB):
            return False
        #actually try to query
        x_index, y_index, theta_index = self.coordinates_to_index(car_frame_goal_state)
        return self.is_reachables[x_index, y_index, theta_index]

    def plan_collision_free_path_in_set(self, car_frame_goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        Does NOT check for collisions!
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        # if not self.contains(goal_state):
        #     return (np.inf, None)
        # return self.planner(self.state, goal_state)
        assert(self.contains(car_frame_goal_state))
        x_index, y_index, theta_index = self.coordinates_to_index(car_frame_goal_state)
        return self.costs[x_index,y_index,theta_index], self.index_to_coordinates(x_index,y_index,theta_index)

    def find_closest_state(self, car_frame_query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param car_frame_query_point:
        :return: Tuple (closest_state, closest_point_is_self.state)
        '''
        closest_state = np.zeros(3)
        closest_state[0] = min(max(car_frame_query_point[0], self.x_range[0]), self.x_range[1])
        closest_state[1] = min(max(car_frame_query_point[1], self.y_range[0]), self.y_range[1])
        closest_state[2] = wrap_angle(car_frame_query_point[2])
        return closest_state, np.all(closest_state==self.origin_index)

    def index_to_coordinates(self, i,j,k):
        assert(0<= i < self.x_count and 0 <= j <self.y_count and 0 <= k <self.theta_count)
        x = self.x_range[0]+i*self.x_resolution
        y = self.y_range[0]+j*self.y_resolution
        theta = wrap_angle(k*self.theta_resolution-np.pi)
        return np.asarray([x,y,theta])

    def coordinates_to_index(self, car_frame_state):
        x_i = min(max(int(np.floor((car_frame_state[0]-self.x_range[0]) / self.x_resolution)), 0), self.x_count - 1)
        y_i = min(max(int(np.floor((car_frame_state[1]-self.y_range[0]) / self.y_resolution)), 0), self.y_count - 1)
        theta_i = min(max(int(np.floor((wrap_angle(car_frame_state[2]) + np.pi) / self.theta_resolution)), 0), self.theta_count - 1)
        return np.asarray([x_i,y_i,theta_i])

    def compute_dubin_path_to_state(self, car_frame_state):
        #compute dubin's path
        path = dubins.shortest_path(np.zeros(3), car_frame_state, self.turn_radius)
        cost = path.path_length()
        #check for undesirable paths: ones that requires turning over pi/2
        step_size = cost/20 #FIXME: arbitrary number
        samples = path.sample_many(step_size)[0]
        assert(cost is not None)
        for s in samples:
            if abs(wrap_angle(s[2]))>np.pi/2:
                return False, np.inf
        return True, cost

    def compute_base_reachable_set(self):
        print('Computing base reachable set...')
        start_time = clock()
        for i in range(self.x_count):
            print('Completed %f %% in %f seconds' %(100*(i*1./self.x_count),(clock()-start_time)))
            for j in range(self.y_count):
                for k in range(self.theta_count):
                    self.is_reachables[i, j, k], self.costs[i, j, k] = self.compute_dubin_path_to_state(self.index_to_coordinates(i, j, k))
        print('Completed computation after %f seconds' %(clock()-start_time))

if __name__=='__main__':
    base_dc_reachable_set = Base_DC_Reachable_Set()
    print('Storing file...')
    start_time = clock()
    np.save('precomputation_results/brs_is_reachables', base_dc_reachable_set.is_reachables)
    np.save('precomputation_results/brs_costs',base_dc_reachable_set.costs)
    print('Stored file after %f seconds' % (clock() - start_time))

    # #For testing
    # for foo in range(100):
    #     x = np.random.rand(1)*5
    #     y = (np.random.rand(1)-0.5)*10
    #     theta = (np.random.rand(1)-0.5)*2*np.pi
    #     i,j,k = base_dc_reachable_set.coordinates_to_index(np.asarray([x,y,theta]))
    #     x_p,y_p,theta_p = base_dc_reachable_set.index_to_coordinates(i,j,k)
    #     try:
    #         assert(abs(x-x_p)<=base_dc_reachable_set.x_resolution and abs(y-y_p)<= base_dc_reachable_set.y_resolution
    #                 and abs(angle_diff(theta_p,theta))<=base_dc_reachable_set.theta_resolution)
    #
    #     except AssertionError:
    #         print('original', x,y,theta)
    #         print('new', x_p, y_p, theta_p)