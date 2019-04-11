import numpy as np
import pickle
from common.rg_rrt_star import *
import os
from collections import deque
from utils.utils import *
from bounding_box_closest_polytope.lib.box import AABB, point_in_box

class DC_Car_Frame_Segment:
    def __init__(self, start, end, cost):
        '''

        :param start:
        :param end:
        '''
        self.start = start
        self.end = end
        self.end_in_start_frame = self.transform_to_start_frame(self.end)
        self.arc_r = np.linalg.norm(self.end_in_start_frame[0:2])/np.sin(abs(self.end_in_start_frame[2]/2))
        self.cost = cost #TODO

    def transform_from_start_frame(self, state):
        ans = np.zeros(3)
        ans[0] = np.cos(self.start[2]) * state[0] - np.sin(self.start[2]) * state[1]
        ans[1] = np.sin(self.start[2]) * state[0] + np.cos(self.start[2]) * state[1]
        if state.shape[0]==2:
            return ans[0:2]+self.start[0:2]
        else:
            ans[2] = state[2]
            return ans+self.start

    def transform_to_start_frame(self, state):
        '''
        Given a point in the world, transform it to the car frame
        :param state:
        :return:
        '''
        ans = np.zeros(3)

        delta_x = state[0] - self.start[0]
        delta_y = state[1] - self.start[1]

        #calculate x
        ans[0] = np.cos(self.start[2])*delta_x+np.sin(self.start[2])*delta_y
        #calculate y
        ans[1] = -np.sin(self.start[2])*delta_x+np.cos(self.start[2])*delta_y
        #calculate theta
        ans[2] = state[2] - self.start[2]
        #handle theta wrapping around
        ans[2] = wrap_angle(ans[2])
        return ans

    def get_fraction_state(self, fraction):
        '''
        returns the state that represents traversing "fraction" of the path's length
        :param fraction: float in 0~1
        :return: a state
        '''
        assert(0<=fraction<=1)
        ans_in_start_frame = np.zeros(3)
        if self.end_in_start_frame[2]<1e-4: #FIXME: approximation for line
            ans_in_start_frame = self.end_in_start_frame*fraction
        else:
            ans_in_start_frame[0] = self.arc_r*np.sin(abs(self.end_in_start_frame[2]*fraction))
            ans_in_start_frame[1] = (self.arc_r-np.cos(self.arc_r*abs(self.end_in_start_frame[2]*fraction)))*np.sign(self.end_in_start_frame[2])
        return self.transform_from_start_frame(ans_in_start_frame)

class DC_Car_Frame_Path(Path):
    def __init__(self, segments):
        '''
        Creates a path object
        FIXME: This implementation is kinda stupid
        '''
        Path.__init__(self)
        self.segments = segments
        self.costs = np.zeros(len(self.segments))
        for i, s in enumerate(self.segments):
            self.costs[i] = s.cost
        self.cumulative_cost = np.cumsum(self.costs)
        self.total_cost = self.cumulative_cost[-1]

    def __repr__(self):
        pass

    def append(self, path):
        raise('NotImplementedError')

    def get_fraction_state(self, fraction):
        #binary search to find the right segment
        #FIXME: something faster
        assert(0<=fraction<=1)
        cost_value = fraction*self.total_cost
        seg_index = np.argwhere(self.cumulative_cost>cost_value)[0]
        if seg_index==0:
            fraction_in_seg = cost_value/self.cumulative_cost[0]
        else:
            fraction_in_seg = (cost_value-self.cumulative_cost[seg_index-1])/(self.cumulative_cost[seg_index]-self.cumulative_cost[seg_index-1])
        return self.segments[seg_index].get_fraction_state(fraction_in_seg)


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
        reachability = False
        cost = np.inf
        path = None
        goal_state[2] = wrap_angle(goal_state[2])
        #FIXME: Implement a better local planner!
        #Naive one turn Dubin controller
        # print('goal state: ',goal_state)
        if abs(goal_state[2])>np.pi/2:
            #FIXME: Currently doesn't consider the car turning around (over pi/2)
            return reachability, cost, path
        if (goal_state[2]>0 and goal_state[1]<0) or (goal_state[2]<0 and goal_state[1]>0):
            return reachability, cost, path

        x_bound = self.turn_radius*np.sin(abs(goal_state[2]))
        # print(x_bound,y_bound)

        if abs(goal_state[0])<x_bound:
            #compute global y bound

            #compute goal based y bound
            theta = np.arctan(goal_state[0]/self.turn_radius)
            if goal_state[2] >= 0:
                y_bound = self.turn_radius - self.turn_radius * np.cos(theta)
            elif goal_state[2] <= 0:
                y_bound = -self.turn_radius + self.turn_radius * np.cos(theta)

            #remove states
            if goal_state[2]>0:
                if goal_state[1]>y_bound:
                    # print('rejected')
                    return reachability, cost, path
            elif goal_state[2]<0:
                if goal_state[1]<y_bound:
                    # print('rejected')
                    return reachability, cost, path

        goal_x_bound = self.turn_radius - self.turn_radius * np.cos(goal_state[2])
        if goal_state[1]>0:
            if goal_y_bound


        if goal_state[0]<=0.5:
            print(goal_state, x_bound,y_bound)
            # if abs(goal_state[0]) < x_bound and abs(goal_state[2] > np.pi/2):
        #     if abs(goal_state[1])>abs(y_bound):
        #         return reachability, cost, path

        reachability = True
        #calculate cost
        #FIXME: This does not make sense at all
        arc_r = np.linalg.norm(goal_state[0:2])/np.sin(abs(goal_state[2]/2))
        if arc_r > 1e5: #FIXME: Workaround
            start_turn_x = goal_state[0]
        else:
            start_turn_x = goal_state[0]-np.sin(abs(goal_state[2]))*arc_r
        line_cost = start_turn_x
        line_segment = DC_Car_Frame_Segment(np.zeros(3), np.asarray([start_turn_x,0,0]), line_cost)
        if arc_r > 1e5:
            path = DC_Car_Frame_Path([line_segment])

        else:
            arc_cost = arc_r * goal_state[2]
            arc_segment = DC_Car_Frame_Segment(np.asarray([start_turn_x,0,0]), goal_state, arc_cost)
            path = DC_Car_Frame_Path([line_segment,arc_segment])
        return reachability, path, cost

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