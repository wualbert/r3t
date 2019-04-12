'''
RG-RRT*

'''

import numpy as np
from time import clock

class Path:
    #TODO
    def __init__(self):
        pass

    def __repr__(self):
        raise ('NotImplementedError')

    def append(self, path):
        raise ('NotImplementedError')

class ReachableSet:
    '''
    Base class of ReachableSet
    '''
    def __init__(self,parent_state=None,path_class=None):#, state, planner, in_set, collision_test):
        '''

        :param state: The state this reachable set belongs to
        :param planner: Local planner for planning paths between self.state and any point in the reachable set.
        :param in_set: A fast checker for checking whether a state is in the set
        Planner takes in (state, goal) and returns (cost_to_go, path)
        '''
        self.path_class = path_class
        self.parent_state = parent_state
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

    def plan_collision_free_path_in_set(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
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
        '''
        A node in the RRT tree
        :param state: the state associated with the node
        :param reachable_set: the reachable points from this node
        :param parent: the parent of the node
        :param path_from_parent: the path connecting the parent to state
        :param children: the children of the node
        :param cost_from_parent: cost to go from the parent to the node
        '''
        self.state = state
        self.reachable_set = reachable_set
        self.parent = parent
        self.path_from_parent = path_from_parent
        self.cost_from_parent = cost_from_parent
        if children is not None:
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
                    '   path from parent: ' + self.path_from_parent.__repr__()+'\n'+ \
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

    def __hash__(self):
        return hash(str(self.state))

    def __eq__(self, other):
        return self.__hash__()==other.__hash__()

    def add_children(self, new_hildren_and_paths):
        '''
        adds new children, represented as a set, to the node
        :param new_children:
        :return:
        '''
        self.children.update(new_hildren_and_paths)

    def update_parent(self, new_parent=None, cost_self_from_parent=None, path_self_from_parent=None):
        '''
        updates the parent of the node
        :param new_parent:
        :return:
        '''
        if self.parent is not None and new_parent is not None:  #assigned a new parent
            self.parent.children.remove(self)
            self.parent = new_parent
            self.parent.children.add(self)
        #calculate new cost from root #FIXME: redundancy
        assert(self.parent.reachable_set.contains(self.state))
        cost_self_from_parent, path_self_from_parent = self.parent.reachable_set.plan_collision_free_path_in_set(self.state)
        cost_root_to_parent = self.parent.cost_from_root
        self.cost_from_parent = cost_self_from_parent
        self.cost_from_root = cost_root_to_parent+self.cost_from_parent
        self.path_from_parent = path_self_from_parent
        #calculate new cost for children
        for child in self.children:
            child.update_parent()
        # print(self.parent.state, 'path', self.path_from_parent)
        # assert(np.all(self.parent.state==self.path_from_parent[0]))
        # assert(np.all(self.state==self.path_from_parent[1]))

class ReachableSetTree:
    '''
    Wrapper for a fast data structure that can help querying
    '''
    def __init__(self):
        pass

    def insert(self, id, state):
        raise('NotImplementedError')

    def nearest_k_neighbors(self, query_state, k=1):
        raise('NotImplementedError')

    def d_neighbors(self, query_state, d = np.inf):
        raise('NotImplementedError')

class RGRRTStar:
    def __init__(self, root_state, compute_reachable_set, sampler, reachable_set_tree, path_class, rewire_radius = None):
        '''
        Base RG-RRT*
        :param root_state: The root state
        :param compute_reachable_set: A function that, given a state, returns its reachable set
        :param sampler: A function that randomly samples the state space
        :param reachable_set_tree: A StateTree object for fast querying
        :param path_class: A class handel that is used to represent path
        '''
        self.root_node = Node(root_state, compute_reachable_set(root_state), cost_from_parent=0)
        self.root_id = hash(str(root_state))
        self.state_dim = root_state[0]
        self.compute_reachable_set = compute_reachable_set
        self.sampler = sampler
        self.goal_state = None
        self.goal_node = None
        self.path_class = path_class
        self.reachable_set_tree = reachable_set_tree() #tree for fast node querying
        self.reachable_set_tree.insert(self.root_id, self.root_node.reachable_set)
        self.state_to_node_map = dict()
        self.state_to_node_map[self.root_id] = self.root_node
        self.node_tally = 0
        self.rewire_radius=rewire_radius

    def create_child_node(self, parent_node, child_state,cost_from_parent = None, path_from_parent = None):
        '''
        Given a child state reachable from a parent node, create a node with that child state
        :param parent_node: parent
        :param child_state: state inside parent node's reachable set
        :param cost_from_parent: FIXME: currently unused
        :param path_from_parent: FIXME: currently unused
        :return:
        '''
        # Update the nodes
        # compute the cost to go and path to reach from parent
        # if cost_from_parent is None or path_from_parent is None:
        assert (parent_node.reachable_set.contains(child_state))
        cost_from_parent, path_from_parent = parent_node.reachable_set.plan_collision_free_path_in_set(child_state)

        # construct a new node
        new_node = Node(child_state, self.compute_reachable_set(child_state),
                        parent=parent_node, path_from_parent=path_from_parent, cost_from_parent=cost_from_parent)
        parent_node.children.add(new_node)
        self.node_tally+=1
        return new_node

    def extend(self, new_state, nearest_node):

        # check for obstacles
        cost_to_go, path = nearest_node.reachable_set.plan_collision_free_path_in_set(new_state)
        #FIXME: Support for partial extensions
        if cost_to_go == np.inf:
            return False, None
        new_node = self.create_child_node(nearest_node, new_state, cost_to_go, path)
        return True, new_node

    def build_tree_to_goal_state(self, goal_state, allocated_time = 20, stop_on_first_reach = False):
        #TODO: Timeout and other termination functionalities
        start = clock()
        self.goal_state = goal_state
        goal_node = None
        while True:
            if stop_on_first_reach:
                if self.goal_node is not None:
                    print('Found path to goal in %f seconds after exploring %d nodes' % (
                    clock() - start, self.node_tally))
                    return self.goal_node
            if clock()-start>allocated_time:
                if self.goal_node is None:
                    print('Unable to find path within %f seconds!' % (clock() - start))
                    return None
                else:
                    print('Found path to goal in %f seconds after exploring %d nodes' % (
                    clock() - start, self.node_tally))
                    return self.goal_node

            #sample the state space
            random_sample = self.sampler()
            # map the states to nodes
            nearest_state_id_list = list(self.reachable_set_tree.nearest_k_neighbors(random_sample, k=1))  # FIXME: necessary to cast to list?
            discard = True
            for i, nearest_state_id in enumerate(nearest_state_id_list):
                nearest_node = self.state_to_node_map[nearest_state_id]
                if nearest_node.reachable_set.contains(random_sample):
                    # find the closest state in the reachable set and use it to extend the tree
                    new_state, discard = nearest_node.reachable_set.find_closest_state(random_sample)
                if not discard:
                    break
            if discard:  # No state in the reachable set is better the the nearest state
                continue

            is_extended, new_node = self.extend(new_state, nearest_node)
            if not is_extended: #extension failed
                continue

            #add the new node to the set tree
            new_state_id = hash(str(new_node.state))
            self.reachable_set_tree.insert(new_state_id, new_node.reachable_set)
            self.state_to_node_map[new_state_id] = new_node

            #rewire the tree
            self.rewire(new_node)
            #In "find path" mode, if the goal is in the reachable set, we are done
            if new_node.reachable_set.contains(goal_state): #FIXME: support for goal region
                # check for obstacles
                cost_to_go, path = new_node.reachable_set.plan_collision_free_path_in_set(goal_state)
                if cost_to_go == np.inf:
                    continue
                # add the goal node to the tree
                goal_node = self.create_child_node(new_node, goal_state)
                self.rewire(goal_node)
                self.goal_node=goal_node


    def rewire(self, new_node):
        #FIXME: better bounding radius
        if self.rewire_radius is None:
            ball_radius = 3*new_node.cost_from_parent
        else:
            ball_radius=self.rewire_radius
        rewire_candidate_states = list(self.reachable_set_tree.d_neighbors(new_node.state, ball_radius))
        #rewire the parent of the new node
        best_new_parent = None
        best_cost_to_new_node = new_node.cost_from_root
        for cand_state in rewire_candidate_states:
            parent_candidate_node = self.state_to_node_map[cand_state]
            if parent_candidate_node==new_node.parent or parent_candidate_node==new_node:
                continue
            #check if it's better to connect to new node through the candidate
            if not parent_candidate_node.reachable_set.contains(new_node.state):
                continue
            cost_to_go, path = parent_candidate_node.reachable_set.plan_collision_free_path_in_set(new_node.state)
            if parent_candidate_node.cost_from_root+cost_to_go < best_cost_to_new_node:
                best_new_parent = parent_candidate_node
                best_cost_to_new_node = parent_candidate_node.cost_from_root+cost_to_go

        #update if the best parent is changed
        if best_new_parent is not None:
            new_node.update_parent(best_new_parent)

        #try to make the new node the candidate's parent
        for cand_state in rewire_candidate_states:
            candidate_node = self.state_to_node_map[cand_state]
            if candidate_node == new_node.parent or candidate_node == self.root_node:
                continue
            if not new_node.reachable_set.contains(candidate_node.state):
                continue
            cost_to_go, path = new_node.reachable_set.plan_collision_free_path_in_set(candidate_node.state)
            if candidate_node.cost_from_root > cost_to_go+new_node.cost_from_root:
                candidate_node.update_parent(new_node, cost_to_go, path)
        return True

    def get_root_to_node_path(self, node):
        states = []
        n = node
        while True:
            states.append(n.state)
            n = n.parent
            if n is None:
                break
        return states.reverse()