'''
RG-RRT*

'''

import numpy as np

class ReachableSet:
    '''
    Base class of ReachableSet
    '''
    def __init__(self):#, state, planner, in_set, collision_test):
        '''

        :param state: The state this reachable set belongs to
        :param planner: Local planner for planning paths between self.state and any point in the reachable set.
        :param in_set: A fast checker for checking whether a state is in the set
        Planner takes in (state, goal) and returns (cost_to_go, path)
        '''
        pass
        # self.state = state
        # self.state_dim = state.shape[0]
        # self.planner = planner
        # self.in_set = in_set
        # self.collision_test = collision_test

    def contains(self, goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        raise('NotImplementedError')

    def plan_collision_free_path(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). Cost to go is set to infinity
        '''
        # if not self.contains(goal_state):
        #     return (np.inf, None)
        # return self.planner(self.state, goal_state)
        raise('NotImplementedError')


    def find_closest_state(self, query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        raise('NotImplementedError')

class Node:
    def __init__(self, state, reachable_set, parent = None, path_from_parent = None,
                 children = None, cost_from_parent = np.inf):
        self.state = state
        self.reachable_set = reachable_set
        self.parent = parent
        self.path_from_parent = path_from_parent
        self.cost_from_parent = cost_from_parent
        if children:
            self.children = children
        else:
            self.children = set()
        if self.parent:
            self.cost_from_root = self.parent.cost_from_root + self.cost_from_parent
        else:
            self.cost_from_root = cost_from_parent

    def __repr__(self):
        if self.parent:
            return '\nRG-RRT* Node: '+'\n'+ \
                    '   state: ' + str(self.state) +'\n'+\
                    '   parent state: ' + str(self.parent.state) +'\n'+ \
                    '   cost from parent: ' + str(self.cost_from_parent) + '\n' + \
                    '   cost from root: ' + str(self.cost_from_root) + '\n' #+ \
                    # '   children: ' + self.children.__repr__() +'\n'
        else:
            return '\nRG-RRT* Node: '+'\n'+ \
                    '   state: ' + str(self.state) +'\n'+\
                    '   parent state: ' + str(None) +'\n'+ \
                    '   cost from parent: ' + str(self.cost_from_parent) + '\n' + \
                    '   cost from root: ' + str(self.cost_from_root) + '\n' #+ \
                    # '   children: ' + self.children.__repr__() +'\n'
    def add_children(self, new_hildren_and_paths):
        '''
        adds new children, represented as a set, to the node
        :param new_children:
        :return:
        '''
        self.children.update(new_hildren_and_paths)

    def update_parent(self, new_parent=None):
        '''
        updates the parent of the node
        :param new_parent:
        :return:
        '''
        if self.parent:
            self.parent = new_parent
        #calculate new cost from root
        cost_self_from_parent, path_self_from_parent = self.parent.reachable_set.plan_collision_free_path(self.state)
        cost_root_to_parent = self.parent.cost_from_root
        self.cost_from_parent = cost_self_from_parent
        self.cost_from_root = cost_root_to_parent+self.cost_from_parent
        self.path_from_parent = path_self_from_parent
        #calculate new cost for children
        for child in self.children:
            child.update_parent()

class StateTree:
    def __init__(self):
        pass

    def insert(self, id, state):
        raise('NotImplementedError')

    def nearest_k_neighbors(self, query_state, k=1):
        raise('NotImplementedError')

    def d_neighbors(self, query_state, d = np.inf):
        raise('NotImplementedError')

class RGRRTStar:
    def __init__(self, root_state, compute_reachable_set, sampler, state_tree):
        '''
        Base RG-RRT*
        :param root_state: The root state
        :param compute_reachable_set: A function that, given a state, returns its reachable set
        :param sampler: A function that randomly samples the state space
        :param state_tree: A StateTree object for fast querying
        '''
        self.root_node = Node(root_state, compute_reachable_set(root_state), cost_from_parent=0)
        self.root_id = hash(str(root_state))
        self.state_dim = root_state.shape[0]
        self.compute_reachable_set = compute_reachable_set
        self.sampler = sampler
        self.state_tree = state_tree


        self.state_tree = state_tree #tree for fast node querying
        self.state_tree.insert(self.root_id,root_state)
        self.state_to_node_map = dict()
        self.state_to_node_map[self.root_id] = self.root_node

    def create_child_node(self, parent_node, child_state):
        # Update the nodes
        # compute the cost to go and path to reach from parent
        cost_from_parent, path_from_parent = parent_node.reachable_set.plan_collision_free_path(child_state)

        # compute the reachable set from the new state
        new_reachable_set = self.compute_reachable_set(child_state)

        # construct a new node
        new_node = Node(child_state, self.compute_reachable_set(child_state),
                        parent=parent_node, path_from_parent=path_from_parent, cost_from_parent=cost_from_parent)

        return new_node

    def build_tree_to_goal_state(self, goal_state, timeout = False):
        #TODO: Timeout and other termination functionalities
        goal_node = None
        while True:
            #extend the tree
            random_sample = self.sampler()

            #find the nearest state in   the tree
            nearest_state_id = list(self.state_tree.nearest_k_neighbors(random_sample, k=1))[0] #FIXME: necessary to cast to list?

            #map the states to nodes
            nearest_node = self.state_to_node_map[nearest_state_id]

            #find the closest state in the reachable set and use it to extend the tree
            new_state, discard = nearest_node.reachable_set.find_closest_state(random_sample)
            if discard: #No state in the reachable set is better the the nearest state
                continue

            new_node = self.create_child_node(nearest_node, new_state)

            #rewire the tree
            self.rewire(new_node)


            #add the new node to the state tree
            new_state_id = hash(str(new_state))
            self.state_tree.insert(new_state_id, new_state)
            self.state_to_node_map[new_state_id] = new_node

            #In "find path" mode, if the goal is in the reachable set, we are done
            if new_node.reachable_set.contains(goal_state) and goal_node is None and not timeout:
                goal_node = self.create_child_node(new_node, goal_state)
                self.rewire(goal_node)
                break

        return goal_node

    def rewire(self, new_node):
        #FIXME: better bounding radius
        ball_radius = 2*new_node.cost_from_parent

        rewire_candidates = list(self.state_tree.d_neighbors(new_node.state, ball_radius))
        best_node = new_node
        for cand in rewire_candidates:
            candidate_node = self.state_to_node_map[cand]
            #attempts to connect the
            cand_child_node = self.create_child_node(candidate_node, new_node.state)
            if cand_child_node.cost_from_root < best_node.cost_from_root:
                best_node = cand_child_node
        #update if the best parent is changed
        if best_node!=new_node:
            new_node.update_parent(best_node.parent)
        return

    def get_root_to_node_path(self, node):
        states = []
        n = node
        while True:
            states.append(n.state)
            n = n.parent
            if n is None:
                break
        return states.reverse()