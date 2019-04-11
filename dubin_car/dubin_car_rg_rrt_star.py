from common.rg_rrt_star import *
from rtree import index
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
import pickle
import os
from utils.utils import *
from base_reachable_set import Base_DC_Reachable_Set

from bounding_box_closest_polytope.lib.box import AABB, point_in_box
from bounding_box_closest_polytope.visualization.visualize import visualize_boxes

class DC_Path(Path):
    def __init__(self,state, car_frame_path):
        '''
        Creates a path object using base path from car frame and a state
        :param state:
        :param base_path: DC_Car_Frame_Path object
        '''
        Path.__init__(self)
        raise ('NotImplementedError')

# load base reachable set
dir_path = os.path.dirname(os.path.realpath(__file__))
with open(dir_path + '/base_reachable_set.p', 'rb') as f:
    BASE_REACHABLE_SET = pickle.load(f)

class DC_ReachableSet(ReachableSet):
    '''
    Base class of ReachableSet
    '''
    def __init__(self,state):#, state, planner, in_set, collision_test):
        '''

        :param state: The state this reachable set belongs to
        :param base_set: The base reachable set corresponding to car at (x,y,theta)=(0,0,0)
        Planner takes in (state, goal) and returns (cost_to_go, path)
        '''
        ReachableSet.__init__(self,DC_Path)
        self.state = state
        #pull up base set
        self.base_set = BASE_REACHABLE_SET
        self.AABB = None
        self.get_AABB()
        self.id = hash(str(self.state))

    def get_AABB(self):
        '''
        Constructs the axis aligned bounding box of the reachable set as an AABB object
        :return:
        '''
        #TODO
        vertices = np.zeros([4,2])
        vertices[0,:] = self.AABB.u
        vertices[1,:] = self.AABB.v
        vertices[2,0] = self.AABB.u[0]
        vertices[2,1] = self.AABB.v[1]
        vertices[3,0] = self.AABB.u[1]
        vertices[3,1] = self.AABB.v[0]
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
            world_frame_point[2] = car_frame_point[2]
            return world_frame_point+self.state

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
        return self.base_set.contains(self.transform_to_car_frame(goal_state))

    def plan_collision_free_path_in_set(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        cost_to_go, car_frame_path = self.base_set.plan_collision_free_path_in_set(self.transform_to_car_frame(goal_state))
        return cost_to_go, DC_Path(self.state, car_frame_path)


    def find_closest_state(self, query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        car_frame_query_point = query_point
        return self.transform_from_car_frame(self.base_set.find_closest_state(car_frame_query_point))

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

    def compute_reachable_set(self, state):
        return DC_ReachableSet(state)

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

class DC_ReachableSetTree(ReachableSetTree):
    '''
    Wrapper for a fast data structure that can help querying
    FIXME: The tree is projected onto 2D due to rtree limitations
    '''
    def __init__(self):
        ReachableSetTree.__init__(self)
        self.tree = index.Index()
        self.reachable_set_dict = {}

    def insert(self, id, reachable_set):
        self.tree.insert(id, [reachable_set.AABB.u[0],reachable_set.AABB.u[1],reachable_set.AABB.v[0], reachable_set.AABB.v[1]])
        self.reachable_set_dict[id] = reachable_set

    def nearest_k_neighbors(self, query_state, k=1):
        nearest_ids = list(self.tree.nearest([query_state[0],query_state[1],query_state[0],query_state[1]], k))
        nearest_ids.sort(key = lambda id: abs(angle_diff(self.reachable_set_dict[id][2], query_state[2]))) #sort nearest state ids by theta distance
        return nearest_ids[0:min(k, len(nearest_ids))]

    def d_neighbors(self, query_state, d = np.inf):
        return self.tree.intersection([query_state[0]-d/2,query_state[1]-d/2,query_state[0]+d/2,query_state[1]+d/2], objects=False)

class DC_RGRRTStar(RGRRTStar):
    def __init__(self, root_state,rewire_radius):
        RGRRTStar.__init__(self, root_state, DC_Map.compute_reachable_set, DC_Map.random_sampler, DC_ReachableSetTree, DC_Path, rewire_radius)

    def visualize(self):
        pass