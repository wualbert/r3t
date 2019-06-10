import pydrake
from rg_rrt_star.common.rg_rrt_star import *
from polytope_symbolic_system.common.symbolic_system import *
from closest_polytope.pypolycontain.lib.AH_polytope import distance_point
from rtree import index #fixme
from collections import deque

class PolytopeReachableSet(ReachableSet):
    def __init__(self, polytope, epsilon=1e-3):
        self.polytope = polytope
        self.epsilon = epsilon
        self.parent_distance = distance_point(self.polytope, self.parent_state)
        assert(self.parent_distance<self.epsilon)

    def contains(self, goal_state):
        if distance_point(self.polytope, goal_state) < self.epsilon:
            return True
        return False

    def find_closest_state(self, query_point):
        '''
        Find the closest state from the query point to a given polytope
        :param query_point:
        :return:
        '''


class PolytopePath:
    def __init__(self):
        self.path = deque()

    def __repr__(self):
        return str(self.path)
    def append(self, path):
        self.path.append(path)

class PolytopeReachableSetTree(ReachableSetTree):
    '''
    Naive implementation of polytope tree
    '''
    def __init__(self):
        ReachableSetTree.__init__(self)
        self.polytope_reachable_sets = {}

    def insert(self, id, reachable_set):
        self.polytope_reachable_sets[id] = reachable_set

    def nearest_k_neighbor_ids(self, query_state, k=1):
        #FIXME: this implementation is slow
        if k>1:
            raise NotImplementedError
        else:
            best_id = None
            best_distance = np.inf
            for id in self.polytope_reachable_sets.keys():
                d = distance_point(self.polytope_reachable_sets[id].polytope, query_state)
                if d<best_distance:
                    best_id = id
                    best_distance = d
            return [best_id]

    def d_neighbor_ids(self, query_state, d = np.inf):
        raise('NotImplementedError')

class ContinuousSystem_StateTree(StateTree):
    def __init__(self):
        StateTree.__init__(self)
        self.tree = index.Index()
        self.state_dict = {}

    def insert(self, state_id, state):
        self.tree.insert(state_id, np.asarray([state[0], state[1],state[0], state[1]]))
        self.state_dict[state_id] = state

    def state_ids_in_reachable_set(self, query_reachable_set):
        raise('NotImplementedError')


class ContinuousSystem_RGRRTStar(RGRRTStar):
    def __init__(self, sys, sampler, step_size):
        self.sys = sys
        self.step_size = step_size
        def compute_reachable_set(state):
            '''
            Compute zonotopic reachable set using the system
            :param h:
            :return:
            '''
            reachable_set_polytope = self.sys.get_reachable_zonotope(state)
            return PolytopeReachableSet(reachable_set_polytope)
        RGRRTStar.__init__(self, self.sys.get_current_state(), compute_reachable_set, sampler, PolytopeReachableSetTree, ContinuousSystem_StateTree, PolytopePath)
