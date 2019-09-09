import pydrake
from rg_rrt_star.common.rg_rrt_star import *
from polytope_symbolic_system.common.symbolic_system import *
# from pypolycontain.lib.AH_polytope import distance_point
from pypolycontain.lib.operations import distance_point_polytope as distance_point
from collections import deque
from rtree import index
from closest_polytope.bounding_box.polytope_tree import PolytopeTree
from closest_polytope.bounding_box.box import AH_polytope_to_box, \
    point_to_box_dmax, point_to_box_distance


class PolytopeReachableSet(ReachableSet):
    def __init__(self, parent_state, polytope_list, sys, epsilon=1e-3, contains_goal_function = None, deterministic_next_state = None, use_true_reachable_set=False, reachable_set_step_size=None):
        ReachableSet.__init__(self, parent_state=parent_state, path_class=PolytopePath)
        self.polytope_list = polytope_list
        try:
            self.aabb_list = [AH_polytope_to_box(p, return_AABB=True) for p in self.polytope_list]
        except TypeError:
            self.aabb_list = None
        self.epsilon = epsilon
        self.deterministic_next_state = deterministic_next_state
        self.sys=sys
        self.use_true_reachable_set = use_true_reachable_set
        self.reachable_set_step_size = reachable_set_step_size
        # try:
        #     self.parent_distance = min([distance_point(p, self.parent_state)[0] for p in self.polytope_list])
        # except TypeError:
        #     self.parent_distance = distance_point(self.polytope_list, self.parent_state)[0]

        self.contains_goal_function = contains_goal_function
        # assert(self.parent_distance<self.epsilon)

    def contains(self, goal_state, return_closest_state = True):
        # print(distance_point(self.polytope, goal_state)[0])
        # print(self.polytope)
        try:
            #multimodal
            distance = np.inf
            closest_state = None
            for i, polytope in enumerate(self.polytope_list):
                # if point_to_box_distance(goal_state, self.aabb_list[i])>0:
                #     continue
                current_distance, current_closest_state = distance_point(polytope, goal_state)
                if current_distance < self.epsilon:
                    if return_closest_state:
                        return True, goal_state
                    else:
                        return True
                else:
                    if current_distance < distance:
                        distance = current_distance
                        closest_state = current_closest_state
            return False, closest_state
        except TypeError:
            distance, closest_state = distance_point(self.polytope_list, goal_state)
            if distance < self.epsilon:
                if return_closest_state:
                    return True, closest_state
                else:
                    return True
            if return_closest_state:
                return False, closest_state
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
        p_used = None
        try:
            #use AABB to upper bound distance
            min_dmax = np.inf
            for i, aabb in enumerate(self.aabb_list):
                dmax = point_to_box_dmax(query_point, aabb)
                if dmax < min_dmax:
                    min_dmax = dmax

            for i, p in enumerate(self.polytope_list):
                #ignore polytopes that are impossible
                if point_to_box_distance(query_point, self.aabb_list[i]) > min_dmax:
                    continue
                d, proj = distance_point(p, query_point)
                if d<distance:
                    distance = d
                    closest_point = proj
                    p_used = p
            assert closest_point is not None
        except TypeError:
            closest_point = distance_point(self.polytope_list, query_point)[1]
            p_used = self.polytope_list
        if np.linalg.norm(closest_point-self.parent_state)<self.epsilon:
            return np.ndarray.flatten(closest_point), True
        if self.use_true_reachable_set and self.reachable_set_step_size:
            #solve for the control input that leads to this state
            u = np.dot(np.linalg.pinv(p_used.T), closest_point-p_used.t)[0:self.sys.u.shape[0]]
            # print(u)
            #simulate nonlinear forward dynamics
            state = self.parent_state
            for step in range(int(self.reachable_set_step_size/1e-3)):
                state = self.sys.forward_step(u=np.atleast_1d(u), linearlize=False, modify_system=False, step_size = 1e-2, return_as_env = False,
                     starting_state= state)
            # print(state, closest_point)
            return np.ndarray.flatten(state), False
        else:
            return np.ndarray.flatten(closest_point), False

    def plan_collision_free_path_in_set(self, goal_state, return_deterministic_next_state = False):
        try:
            if self.plan_collision_free_path_in_set_function:
                return self.plan_collision_free_path_in_set_function(goal_state, return_deterministic_next_state)
        except AttributeError:
            pass
        #fixme: support collision checking

        #
        # is_contain, closest_state = self.contains(goal_state)
        # if not is_contain:
        #     print('Warning: this should never happen')
        #     return np.linalg.norm(self.parent_state-closest_state), deque([self.parent_state, closest_state]) #FIXME: distance function

        # Simulate forward dynamics if there can only be one state in the next timestep
        if not return_deterministic_next_state:
            return np.linalg.norm(self.parent_state-goal_state), deque([self.parent_state, goal_state])
        return np.linalg.norm(self.parent_state-goal_state), deque([self.parent_state, goal_state]), self.deterministic_next_state

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
                self.polytope_tree = PolytopeTree(np.atleast_1d([reachable_set.polytope_list]).flatten(), key_vertex_count=self.key_vertex_count)
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
            # assert(len(self.polytope_tree.find_closest_polytopes(query_state))==1)
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
        self.state_idx = None

    # delayed initialization to consider dimensions
    def initialize(self, dim):
        self.state_tree_p.dimension=dim
        print('Symbolic System State Tree dimension is %d-D' % self.state_tree_p.dimension)
        self.state_idx = index.Index(properties=self.state_tree_p)

    def insert(self, state_id, state):
        if not self.state_idx:
            self.initialize(state.shape[0])
        self.state_idx.insert(state_id, np.concatenate([state,state]))
        self.state_id_to_state[state_id] = state

    def state_ids_in_reachable_set(self, query_reachable_set):
        assert(self.state_idx is not None)
        try:
            state_ids_list = []
            for p in query_reachable_set.polytope_list:
                lu = AH_polytope_to_box(p)
                state_ids_list.extend(list(self.state_idx.intersection(lu)))
            return state_ids_list
        except TypeError:
            lu = AH_polytope_to_box(query_reachable_set.polytope_list)
            return list(self.state_idx.intersection(lu))

class SymbolicSystem_RGRRTStar(RGRRTStar):
    def __init__(self, sys, sampler, step_size, contains_goal_function = None, compute_reachable_set=None, use_true_reachable_set=False):
        self.sys = sys
        self.step_size = step_size
        self.contains_goal_function = contains_goal_function
        if compute_reachable_set is None:
            def compute_reachable_set(state):
                '''
                Compute zonotopic reachable set using the system
                :param h:
                :return:
                '''
                deterministic_next_state = None
                reachable_set_polytope = self.sys.get_reachable_polytopes(state, step_size=self.step_size)
                if np.all(self.sys.get_linearization(state=state).B == 0):
                    deterministic_next_state = self.sys.forward_step(starting_state=state, modify_system=False, return_as_env=False, step_size=self.step_size)
                return PolytopeReachableSet(state,reachable_set_polytope, sys=self.sys, contains_goal_function=self.contains_goal_function, \
                                            deterministic_next_state=deterministic_next_state, reachable_set_step_size=self.step_size, use_true_reachable_set=use_true_reachable_set)
        RGRRTStar.__init__(self, self.sys.get_current_state(), compute_reachable_set, sampler, PolytopeReachableSetTree, SymbolicSystem_StateTree, PolytopePath)