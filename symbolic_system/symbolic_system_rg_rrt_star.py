import pydrake
from rg_rrt_star.common.rg_rrt_star import *
from polytope_symbolic_system.common.symbolic_system import *
from pypolycontain.lib.AH_polytope import distance_point
from collections import deque
from rtree import index
from closest_polytope.bounding_box.polytope_tree import PolytopeTree
from closest_polytope.bounding_box.box import AH_polytope_to_box, zonotope_to_box#FIXME


class PolytopeReachableSet(ReachableSet):
    def __init__(self, parent_state, polytope_list, epsilon=1e-3, contains_goal_function = None):
        ReachableSet.__init__(self, parent_state=parent_state, path_class=PolytopePath)
        self.polytope_list = polytope_list
        self.epsilon = epsilon
        try:
            self.parent_distance = min([distance_point(p, self.parent_state)[0] for p in self.polytope_list])
        except TypeError:
            self.parent_distance = distance_point(self.polytope_list, self.parent_state)[0]

        self.contains_goal_function = contains_goal_function
        # assert(self.parent_distance<self.epsilon)

    def contains(self, goal_state):
        # print(distance_point(self.polytope, goal_state)[0])
        # print(self.polytope)
        try:
            #multimodal
            for polytope in self.polytope_list:
                if distance_point(polytope, goal_state)[0] < self.epsilon:
                    return True
            return False
        except TypeError:
            if distance_point(self.polytope_list, goal_state)[0] < self.epsilon:
                return True
            return False

    def contains_goal(self, goal_state):
        if self.contains_goal is None:
            return self.contains(goal_state)
        else:
            return self.contains_goal_function(self, goal_state)

    def find_closest_state(self, query_point):
        '''
        Find the closest state from the query point to a given polytope
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        distance = np.inf
        closest_point = None
        try:
            for p in self.polytope_list:
                d, proj = distance_point(p, query_point)
                if d<distance:
                    distance = d
                    closest_point = distance_point(p, query_point)[1]
            assert closest_point is not None
        except TypeError:
            closest_point = distance_point(self.polytope_list, query_point)[1]
        closest_point = np.ndarray.flatten(closest_point)
        return closest_point, np.linalg.norm(closest_point-self.parent_state)<self.epsilon

    def plan_collision_free_path_in_set(self, goal_state):
        #fixme: correct cost function
        if not self.contains(goal_state):
            return np.linalg.norm(self.parent_state-goal_state), None #FIXME: distance function
        return np.linalg.norm(self.parent_state-goal_state), deque([self.parent_state, goal_state])


class PolytopePath:
    def __init__(self):
        self.path = deque()
    def __repr__(self):
        return str(self.path)
    def append(self, path):
        self.path+=path #TODO

class PolytopeReachableSetTree(ReachableSetTree):
    '''
    Polytopic reachable set with PolytopeTree
    '''
    def __init__(self, key_vertex_count = 0):
        self.polytope_tree = None
        self.id_to_reachable_sets = {}
        self.polytope_to_id = {}
        self.key_vertex_count = key_vertex_count
        # for d_neighbor_ids
        # self.state_id_to_state = {}
        # self.state_idx = None
        # self.state_tree_p = index.Property()

    def insert(self, state_id, reachable_set):
        try:
            iter(reachable_set.polytope_list)
            if self.polytope_tree is None:
                self.polytope_tree = PolytopeTree(np.array(reachable_set.polytope_list),
                                                  key_vertex_count=self.key_vertex_count)
                # for d_neighbor_ids
                # self.state_tree_p.dimension = to_AH_polytope(reachable_set.polytope[0]).t.shape[0]
            else:
                self.polytope_tree.insert(np.array(reachable_set.polytope_list))
            self.id_to_reachable_sets[state_id] = reachable_set
            for p in reachable_set.polytope_list:
                self.polytope_to_id[p] = state_id

        except TypeError:
            if self.polytope_tree is None:
                self.polytope_tree = PolytopeTree(np.array([reachable_set.polytope_list]), key_vertex_count=self.key_vertex_count)
                # for d_neighbor_ids
                # self.state_tree_p.dimension = to_AH_polytope(reachable_set.polytope[0]).t.shape[0]
            else:
                self.polytope_tree.insert(np.array([reachable_set.polytope_list]))
            self.id_to_reachable_sets[state_id] = reachable_set
            self.polytope_to_id[reachable_set.polytope_list] = state_id
        # for d_neighbor_ids
        # state_id = hash(str(reachable_set.parent_state))
        # self.state_idx.insert(state_id, np.repeat(reachable_set.parent_state, 2))
        # self.state_id_to_state[state_id] = reachable_set.parent_state

    def nearest_k_neighbor_ids(self, query_state, k=1):
        if k>1:
            raise NotImplementedError
        else:
            if self.polytope_tree is None:
                return None
            best_polytope = self.polytope_tree.find_closest_polytopes(query_state)[0]
            return [self.polytope_to_id[best_polytope]]

    def d_neighbor_ids(self, query_state, d = np.inf):
        '''

        :param query_state:
        :param d:
        :return:
        '''
        # return self.state_idx.intersection(, objects=False)
        raise NotImplementedError

class SymbolicSystem_StateTree(StateTree):
    def __init__(self):
        StateTree.__init__(self)
        self.state_id_to_state = {}
        self.state_tree_p = index.Property()
        self.state_idx = index.Index()

    def insert(self, state_id, state):
        self.state_idx.insert(state_id, np.concatenate([state,state]))
        self.state_id_to_state[state_id] = state

    def state_ids_in_reachable_set(self, query_reachable_set):
        try:
            state_ids_list = []
            for p in query_reachable_set.polytope_list:
                lu = zonotope_to_box(p) #FIXME: change to AH polytope to box; Memoize the polytope's AABB
                state_ids_list.extend(list(self.state_idx.intersection(lu)))
            return state_ids_list
        except TypeError:
            lu = zonotope_to_box(query_reachable_set.polytope_list)
            return list(self.state_idx.intersection(lu))

class SymbolicSystem_RGRRTStar(RGRRTStar):
    def __init__(self, sys, sampler, step_size, contains_goal_function = None):
        self.sys = sys
        self.step_size = step_size
        self.contains_goal_function = contains_goal_function
        def compute_reachable_set(state):
            '''
            Compute zonotopic reachable set using the system
            :param h:
            :return:
            '''
            reachable_set_polytope = self.sys.get_reachable_zonotopes(state, step_size=self.step_size)
            return PolytopeReachableSet(state,reachable_set_polytope, contains_goal_function=self.contains_goal_function)
        RGRRTStar.__init__(self, self.sys.get_current_state(), compute_reachable_set, sampler, PolytopeReachableSetTree, SymbolicSystem_StateTree, PolytopePath)