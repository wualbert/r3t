import numpy as np
# import cPickle as pickle
# import dill
from r3t.common.r3t import *
import os
from collections import deque
from utils.utils import *
from closest_polytope_algorithms.bounding_box.box import AABB, point_in_box
import dubins
from timeit import default_timer

def wrap_angle(theta):
    theta %= 2*np.pi
    if theta>=np.pi:
        return theta-2*np.pi
    else:
        return theta

class Base_DC_Reachable_Set(ReachableSet):
    '''
    A base reachable set for a Dubin's car located at (x,y,theta) = (0,0,0)
    '''
    def __init__(self, x_range=np.asarray([0,20]), y_range=np.asarray([-8,8]), x_count=200,
                 y_count=160, theta_count=150, turn_radius = 2.5, is_reachables= None, costs = None, closest_index = None):
        ReachableSet.__init__(self)
        self.x_range = x_range
        self.y_range = y_range
        self.x_count = x_count
        self.y_count = y_count
        self.theta_count = theta_count
        self.x_resolution = (self.x_range[1]-self.x_range[0])*1./self.x_count
        self.y_resolution = (self.y_range[1]-self.y_range[0])*1./self.y_count
        self.theta_resolution = 2.*np.pi/self.theta_count
        self.turn_radius = turn_radius
        self.AABB = AABB(([self.x_range[0],self.y_range[0]],[self.x_range[1],self.y_range[1]]))
        self.origin_index = self.coordinates_to_index(np.zeros(3))
        if is_reachables is None or costs is None or closest_index is None:
            self.is_reachables = np.empty([self.x_count, self.y_count, self.theta_count], dtype=bool)
            self.costs = np.empty([self.x_count,self.y_count,self.theta_count],dtype=float)
            self.closest_reachable_index = np.empty([self.x_count, self.y_count, self.theta_count], dtype=(float, 3))
            self.compute_base_reachable_set()
        else:
            self.is_reachables=is_reachables
            self.costs=costs
            self.closest_reachable_index = closest_index

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

    def plan_collision_free_path_in_set(self, car_frame_goal_state, return_deterministic_next_state=False):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        Does NOT check for collisions!
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        if not self.contains(car_frame_goal_state):
            # print('This should never happen...')
            # print('tested goal state:', car_frame_goal_state)
            # print('self.contains', self.contains(car_frame_goal_state))
            # print('self.find_closest_state', self.find_closest_state(car_frame_goal_state))
            # print('self.is_reachable', self.is_reachables[self.find_closest_state(car_frame_goal_state)])
            return (np.inf, None)
        assert(self.contains(car_frame_goal_state))
        x_index, y_index, theta_index = self.coordinates_to_index(car_frame_goal_state)
        return self.costs[x_index,y_index,theta_index], self.index_to_coordinates(x_index,y_index,theta_index)

    def find_closest_state(self, car_frame_query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param car_frame_query_point:
        :return: Tuple (closest_state, closest_point_is_self.state)
        '''
        # closest_projection = np.zeros(3)
        # closest_projection[0] = min(max(car_frame_query_point[0], self.x_range[0]), self.x_range[1])
        # closest_projection[1] = min(max(car_frame_query_point[1], self.y_range[0]), self.y_range[1])
        # closest_projection[2] = wrap_angle(car_frame_query_point[2])
        i,j,k = self.coordinates_to_index(car_frame_query_point)
        cri = self.closest_reachable_index[i, j, k]

        return self.index_to_coordinates(*cri), np.all(cri==self.origin_index)

    def index_to_coordinates(self, i,j,k):
        assert(0<= i < self.x_count and 0 <= j <self.y_count and 0 <= k <self.theta_count)
        x = self.x_range[0]+i*self.x_resolution
        y = self.y_range[0]+j*self.y_resolution
        theta = wrap_angle(-np.pi+k*self.theta_resolution)
        return np.asarray([x,y,theta])

    def coordinates_to_index(self, car_frame_state):
        x_i = min(max(int(np.round((car_frame_state[0]-self.x_range[0]) / self.x_resolution)), 0), self.x_count - 1)
        y_i = min(max(int(np.round((car_frame_state[1]-self.y_range[0]) / self.y_resolution)), 0), self.y_count - 1)
        theta_i = min(max(int(np.round((car_frame_state[2] + np.pi) / self.theta_resolution)), 0), self.theta_count - 1)
        return np.asarray([x_i,y_i,theta_i])

    def compute_dubin_path_to_state(self, car_frame_state):
        #compute dubin's path
        path = dubins.shortest_path(np.zeros(3), car_frame_state, self.turn_radius)
        cost = path.path_length()
        #check for undesirable paths: ones that requires turning over pi/2
        step_size = cost/100 #FIXME: arbitrary number
        samples = path.sample_many(step_size)[0]
        assert(cost is not None)
        for s in samples:
            if abs(wrap_angle(s[2]))>np.pi/2:
                return False, np.inf
        return True, cost

    def compute_base_reachable_set(self):
        print('Computing base reachable set...')
        start_time = default_timer()
        for i in range(self.x_count):
            print('Completed %f %% in %f seconds' %(100*(i*1./self.x_count),(default_timer()-start_time)))
            for j in range(self.y_count):
                for k in range(self.theta_count):
                    self.is_reachables[i, j, k], self.costs[i, j, k] = self.compute_dubin_path_to_state(self.index_to_coordinates(i, j, k))
        end_time_1 = default_timer()-start_time
        print('Computed base reachable set in %f seconds' %end_time_1)
        print('Processing base reachable set...')
        start_time_2 = default_timer()
        all_reachables = np.argwhere(self.is_reachables)
        shell_reachables = []
        shell_reachable_states = []
        for i in range(all_reachables.shape[0]):
            #only do append internal points
            p,q,r = all_reachables[i,:]
            try:
                if not(self.is_reachables[p+1,q,r] and self.is_reachables[p-1,q,r] and self.is_reachables[p,q,r] and
                       self.is_reachables[p + 1, q+1, r] and self.is_reachables[p - 1, q+1, r] and self.is_reachables[
                           p, q+1, r] and
                       self.is_reachables[p + 1, q-1, r] and self.is_reachables[p - 1, q-1, r] and self.is_reachables[
                           p, q-1, r] and
                       self.is_reachables[p + 1, q, r+1] and self.is_reachables[p - 1, q, r+1] and self.is_reachables[
                           p, q, r+1] and
                       self.is_reachables[p + 1, q+1, r+1] and self.is_reachables[p - 1, q+1, r+1] and self.is_reachables[
                           p, q+1, r+1] and
                       self.is_reachables[p + 1, q-1, r+1] and self.is_reachables[p - 1, q-1, r+1] and self.is_reachables[
                           p, q-1, r+1] and
                       self.is_reachables[p + 1, q+1, r-1] and self.is_reachables[p - 1, q+1, r-1] and self.is_reachables[
                           p, q+1, r-1] and
                       self.is_reachables[p + 1, q, r-1] and self.is_reachables[p - 1, q, r-1] and self.is_reachables[
                           p, q, r-1] and
                       self.is_reachables[p + 1, q-1, r-1] and self.is_reachables[p - 1, q-1, r-1] and self.is_reachables[
                           p, q-1, r-1]):
                    shell_reachables.append(all_reachables[i,:])
                    shell_reachable_states.append(self.index_to_coordinates(p,q,r))
            except:
                pass
        shell_reachables = np.asarray(shell_reachables)
        shell_reachable_states = np.asarray(shell_reachable_states)
        end_time_2 = default_timer()-start_time_2
        print('Processed base reachable set in %f seconds' %end_time_2)

        #compute closest states
        print('Computing closest reachable state...')
        start_time_3=default_timer()
        for i in range(self.x_count):
            print('Completed %f %% in %f seconds' %(100*(i*1./self.x_count),(default_timer()-start_time_3)))
            for j in range(self.y_count):
                for k in range(self.theta_count):
                    closest_ri=np.asarray([i, j, k])
                    if self.is_reachables[i,j,k]:
                        self.closest_reachable_index[i, j, k] = closest_ri
                    else:
                        diff = shell_reachable_states-self.index_to_coordinates(*closest_ri)
                        #L2 norm on (x,y,theta)
                        diff_norm = np.linalg.norm(diff, axis=1)
                        argmin_diff = np.argmin(diff_norm)
                        self.closest_reachable_index[i, j, k] = shell_reachables[argmin_diff]
        end_time_3 = default_timer()-start_time_3
        print('Computed closest reachable state in %f seconds' %end_time_3)
        print('Finished precomputation in %f seconds' %(end_time_1+end_time_2+end_time_3))

if __name__=='__main__':
    base_dc_reachable_set = Base_DC_Reachable_Set()
    print('Storing file...')
    start_time = default_timer()
    np.save('precomputation_results/brs_is_reachables', base_dc_reachable_set.is_reachables)
    np.save('precomputation_results/brs_costs',base_dc_reachable_set.costs)
    np.save('precomputation_results/closest_reachable_index', base_dc_reachable_set.closest_reachable_index)
    print('Stored file after %f seconds' % (default_timer() - start_time))

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