from r3t.common.r3t import *
from rtree import index
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
# import cPickle as pickle
# import dill
import os
from utils.utils import *
import dubin_car.base_reachable_set
import sys
from dubin_car.base_reachable_set import *
import dubins
from time import time
from closest_polytope_algorithms.bounding_box.box import AABB, point_in_box
from closest_polytope_algorithms.visualization.visualize import visualize_boxes

def wrap_angle(theta):
    theta %= 2*np.pi
    if theta>=np.pi:
        return theta-2*np.pi
    else:
        return theta

class DC_Path(Path):
    def __init__(self,state, car_frame_path,turn_radius, cost):
        '''
        Creates a path object using base path from car frame and a state
        :param state:
        :param car_frame_path: 3*2 numpy array representing the car frame path
        '''
        Path.__init__(self)
        self.start_state = state
        self.car_frame_path = car_frame_path
        self.turn_radius = turn_radius
        #Transform to world frame
        self.end_state = self.transform_from_car_frame(car_frame_path)
        self.world_frame_dubins_path = None
        self.cost = cost
    def __repr__(self):
        return ('Dubin path from ' + str(self.start_state) + ' to ' + str(self.end_state))

    def transform_from_car_frame(self, car_frame_point):
        world_frame_point = np.zeros(3)
        world_frame_point[0] = np.cos(self.start_state[2]) * car_frame_point[0] - np.sin(self.start_state[2]) * car_frame_point[1]
        world_frame_point[1] = np.sin(self.start_state[2]) * car_frame_point[0] + np.cos(self.start_state[2]) * car_frame_point[1]
        if car_frame_point.shape[0] == 2:
            return world_frame_point[0:2] + self.start_state[0:2]
        else:
            world_frame_point[0:2] += self.start_state[0:2]
            world_frame_point[2] = wrap_angle(car_frame_point[2]+self.start_state[2])
            return world_frame_point

    def transform_to_car_frame(self, world_frame_point):
        '''
        Given a point in the world, transform it to the car frame
        :param world_frame_point:
        :return:
        '''
        car_frame_point = np.zeros(3)

        delta_x = world_frame_point[0] - self.start_state[0]
        delta_y = world_frame_point[1] - self.start_state[1]

        #calculate x
        car_frame_point[0] = np.cos(self.start_state[2])*delta_x+np.sin(self.start_state[2])*delta_y
        #calculate y
        car_frame_point[1] = -np.sin(self.start_state[2])*delta_x+np.cos(self.start_state[2])*delta_y
        #calculate theta
        car_frame_point[2] = world_frame_point[2] - self.start_state[2]
        #handle theta wrapping around
        car_frame_point[2] = wrap_angle(car_frame_point[2])
        return car_frame_point

    def get_fraction_point(self, fraction):
        '''
        Get the state after traversing fraction of the path
        :param fraction:
        :return:
        '''
        if self.world_frame_dubins_path is None:
            #only compute if necessary
            self.world_frame_dubins_path = dubins.shortest_path(self.start_state, self.end_state, self.turn_radius)
            self.cost = self.world_frame_dubins_path.path_length()
        fraction = max(0,min(1,fraction))
        if fraction>=1-1e-2:
            return self.end_state
        return np.asarray(self.world_frame_dubins_path.sample(fraction * self.cost))

    def cut_short(self, fraction):
        self.end_state = self.get_fraction_point(fraction)
        self.car_frame_path = self.transform_to_car_frame(self.end_state)
        self.world_frame_dubins_path = dubins.shortest_path(self.start_state, self.end_state, self.turn_radius)
        self.cost = self.world_frame_dubins_path.path_length()

    def get_dubins_interpolated_path(self, samples = 20):
        '''
        Get the associated dubin's path expressed as piecewise linear path connected by waypoints
        :param samples: number of waypoints, includes start and end
        :return: numpy array of waypoints
        '''
        fractions = np.linspace(0,1,samples)
        waypoints = np.zeros([samples,3])
        for i in range(samples):
            waypoints[i, :] = self.get_fraction_point(fractions[i])
            # while True:
            #     try:
            #         waypoints[i, :] = self.get_fraction_point(fractions[i])
            #         break
            #     except:
            #         print('Sample point not found!')
            #         print(dubins.shortest_path(np.zeros(3), self.car_frame_path.end, self.turn_radius).path_length())
            #         print(dubins.shortest_path(self.start_state, self.world_frame_end, self.turn_radius).path_length())
            #         print(self.car_frame_dubins_path.path_length())
            #         return np.zeros([samples,3])
        return waypoints

class DC_ReachableSet(ReachableSet):
    '''
    Base class of ReachableSet
    '''
    # load base reachable set
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print('Loading base reachable set...')
    start_time = default_timer()
    brs_is_reachable = np.load(dir_path + '/precomputation_results/brs_is_reachables.npy')
    brs_costs = np.load(dir_path + '/precomputation_results/brs_costs.npy')
    closest_index = np.load(dir_path + '/precomputation_results/closest_reachable_index.npy')

    BASE_REACHABLE_SET = Base_DC_Reachable_Set(is_reachables=brs_is_reachable, costs=brs_costs, closest_index=closest_index)
    print('Loaded base reachable set after %f seconds' % (default_timer() - start_time))


    def __init__(self, state, is_collision):#, state, planner, in_set, collision_test):
        '''

        :param state: The state this reachable set belongs to
        :param base_set: The base reachable set corresponding to car at (x,y,theta)=(0,0,0)
        Planner takes in (state, goal) and returns (cost_to_go, path)
        '''
        ReachableSet.__init__(self,DC_Path)
        self.state = state
        #pull up base set
        self.AABB = None
        self.get_AABB()
        self.id = hash(str(self.state))
        self.turn_radius = self.BASE_REACHABLE_SET.turn_radius
        self.is_collision = is_collision

    def get_AABB(self):
        '''
        Constructs the axis aligned bounding box of the reachable set as an AABB object
        :return:
        '''
        #TODO
        vertices = np.zeros([4,2])
        vertices[0,:] = self.BASE_REACHABLE_SET.x_range[0], self.BASE_REACHABLE_SET.y_range[0]
        vertices[1,:] = self.BASE_REACHABLE_SET.x_range[1], self.BASE_REACHABLE_SET.y_range[1]
        vertices[2,:] =  self.BASE_REACHABLE_SET.x_range[0], self.BASE_REACHABLE_SET.y_range[1]
        vertices[3,:] =  self.BASE_REACHABLE_SET.x_range[1], self.BASE_REACHABLE_SET.y_range[0]
        for i in range(4):
            vertices[i,:] = self.transform_from_car_frame(vertices[i,:])
        max_xy = np.amax(vertices, axis = 0)
        min_xy = np.amin(vertices, axis = 0)
        self.AABB = AABB([min_xy,max_xy])

    def transform_from_car_frame(self, car_frame_point):
        world_frame_point = np.zeros(3)
        world_frame_point[0] = np.cos(self.state[2])*car_frame_point[0]-np.sin(self.state[2])*car_frame_point[1]
        world_frame_point[1] = np.sin(self.state[2])*car_frame_point[0]+np.cos(self.state[2])*car_frame_point[1]
        if car_frame_point.shape[0]==2:
            return world_frame_point[0:2]+self.state[0:2]
        else:
            world_frame_point[0:2] += self.state[0:2]
            world_frame_point[2] = wrap_angle(car_frame_point[2]+self.state[2])
            return world_frame_point

    def transform_to_car_frame(self, world_frame_point):
        '''
        Given a point in the world, transform it to the car frame
        :param world_frame_point:
        :return:
        '''
        car_frame_point = np.zeros(3)

        delta_x = world_frame_point[0] - self.state[0]
        delta_y = world_frame_point[1] - self.state[1]

        #calculate x
        car_frame_point[0] = np.cos(self.state[2])*delta_x+np.sin(self.state[2])*delta_y
        #calculate y
        car_frame_point[1] = -np.sin(self.state[2])*delta_x+np.cos(self.state[2])*delta_y
        #calculate theta
        car_frame_point[2] = world_frame_point[2] - self.state[2]
        #handle theta wrapping around
        car_frame_point[2] = wrap_angle(car_frame_point[2])
        return car_frame_point

    def contains(self, goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        return self.BASE_REACHABLE_SET.contains(self.transform_to_car_frame(goal_state))

    def plan_collision_free_path_in_set(self, goal_state, return_deterministic_next_state=False):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        #TODO: Support for partial planning
        cost_to_go, car_frame_path = self.BASE_REACHABLE_SET.plan_collision_free_path_in_set(self.transform_to_car_frame(goal_state))
        if cost_to_go ==np.inf:
            return np.inf, None
        world_frame_path = DC_Path(self.state, car_frame_path, self.turn_radius, cost=cost_to_go)
        #samples
        sample_num = 50
        samples = np.linspace(0,1,sample_num)
        for i in range(0,sample_num):
            if self.is_collision(world_frame_path.get_fraction_point(samples[i])):
                #collision happened, return the point right before collision
                return np.inf, None
        return cost_to_go, DC_Path(self.state, car_frame_path, self.turn_radius, cost=cost_to_go)


    def find_closest_state(self, query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        car_frame_query_point = self.transform_to_car_frame(query_point)
        closest_state, is_self_state =  self.BASE_REACHABLE_SET.find_closest_state(car_frame_query_point)
        return self.transform_from_car_frame(closest_state), is_self_state

class DC_Map:
    '''
    Obstacles do not have a theta
    theta is represented from -np.pi to np.pi
    '''
    def __init__(self, world_bounding_box):
        self.obstacles = [] #pairs of coordinates ((x1,y1),(x2,y2)
        self.world_bounding_box = world_bounding_box
        self.collision_memo = {}
    def random_sampler(self):
        x_rand = np.random.uniform(self.world_bounding_box.u[0], self.world_bounding_box.v[0])
        y_rand = np.random.uniform(self.world_bounding_box.u[1], self.world_bounding_box.v[1])
        theta_rand = np.random.uniform(-np.pi, np.pi)
        return np.asarray([x_rand,y_rand, theta_rand])

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
    #
    # def compute_reachable_set(self, state):
    #     return DC_ReachableSet(state,self.point_collides_with_obstacle)

    def point_collides_with_obstacle(self, point):
        for obs in self.obstacles:
            # print(point)
            if point_in_box(point[0:2], obs):
                return True
        return False

    def line_collides_with_obstacle(self, start, goal):
        '''
        Assume obstacles are only specified by (x,y) and across all thetas
        :param start:
        :param goal:
        :return:
        '''
        string_rep = (str(start),str(goal))
        if string_rep in self.collision_memo:
            return self.collision_memo[string_rep]
        sample_count = 10 #FIXME
        x_samples = np.linspace(start[0], goal[0], sample_count)
        y_samples = np.linspace(start[1], goal[1], sample_count)
        for i in range(sample_count):
            if self.point_collides_with_obstacle(np.asarray([x_samples[i], y_samples[i]])):
                self.collision_memo[string_rep]=True
                return True
        self.collision_memo[string_rep] = False
        # print(start,goal)
        return False

class DC_StateTree(StateTree):
    def __init__(self):
        StateTree.__init__(self)
        self.tree = index.Index()
        self.state_dict = {}

    def insert(self, state_id, state):
        self.tree.insert(state_id, [state[0], state[1], state[0], state[1]])
        self.state_dict[state_id] = state

    def state_ids_in_reachable_set(self, query_reachable_set):
        nearest_ids = list(self.tree.nearest([query_reachable_set.AABB.u[0],query_reachable_set.AABB.u[1],query_reachable_set.AABB.v[0],query_reachable_set.AABB.v[1]]))
        return nearest_ids


class DC_ReachableSetTree(ReachableSetTree):
    '''
    Wrapper for a fast data structure that can help querying
    FIXME: The tree is projected onto 2D due to rtree limitations
    '''
    def __init__(self):
        ReachableSetTree.__init__(self)
        self.tree = index.Index()
        self.reachable_set_dict = {}

    def insert(self, state_id, reachable_set):
        self.tree.insert(state_id, [reachable_set.AABB.u[0], reachable_set.AABB.u[1], reachable_set.AABB.v[0], reachable_set.AABB.v[1]])
        self.reachable_set_dict[state_id] = reachable_set

    def nearest_k_neighbor_ids(self, query_state, k=1):
        nearest_ids = list(self.tree.nearest([query_state[0],query_state[1],query_state[0],query_state[1]], k))
        # nearest_ids.sort(key = lambda state_id: abs(angle_diff(self.reachable_set_dict[state_id].state[2], query_state[2]))) #sort nearest state ids by theta distance
        nearest_ids.sort(key = lambda state_id: self.reachable_set_dict[state_id].plan_collision_free_path_in_set(query_state)[0])

        return nearest_ids[0:min(k, len(nearest_ids))]

    def d_neighbor_ids(self, query_state):
        return self.tree.intersection([query_state[0],query_state[1],query_state[0],query_state[1]], objects=False)

class DC_R3T(R3T):
    def __init__(self, root_state, point_collides_with_obstacle, random_sampler, rewire_radius):
        '''
        Dubin's car RG-RRT* problem setup.
        :param root_state: The starting position of the car in world/map frame
        :param point_collides_with_obstacle: function handle that computes whether a point collides with an obstacle
        :param random_sampler:
        :param rewire_radius:
        '''
        def compute_reachable_set(state):
            return DC_ReachableSet(state,point_collides_with_obstacle)
        R3T.__init__(self, root_state, compute_reachable_set, random_sampler, DC_ReachableSetTree, DC_StateTree, DC_ReachableSet, rewire_radius)