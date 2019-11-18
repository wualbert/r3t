'''
RG-RRT*

'''

import numpy as np
from timeit import default_timer

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

    def contains_goal(self, goal_state):
        '''
        Containment check for goal state. Supports fuzzy checks.
        :param goal_state:
        :return: contains_goal, path_if_goal_is_contained
        '''
        # if goal is epsilon-away from reachable set, do fine goal check
        # enumerate control inputs to see if goal can be reached
        raise NotImplementedError

    def plan_collision_free_path_in_set(self, goal_state, return_deterministic_next_state=False):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). path is a Path class object
        '''
        # if not self.contains(goal_state):
        #     return (np.inf, None)
        # return self.planner(self.state, goal_state)
        raise('NotImplementedError')


    def find_closest_state(self, query_point, save_true_dynamics_path=False):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        raise('NotImplementedError')

class Node:
    def __init__(self, state, reachable_set, true_dynamics_path, parent = None, path_from_parent = None,
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
        self.true_dynamics_path = true_dynamics_path
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

    def add_children(self, new_children_and_paths):
        #TODO: deprecated this function
        '''
        adds new children, represented as a set, to the node
        :param new_children:
        :return:
        '''
        self.children.update(new_children_and_paths)

    # def update_parent(self, new_parent=None, cost_self_from_parent=None, path_self_from_parent=None):
    #     '''
    #     updates the parent of the node
    #     :param new_parent:
    #     :return:
    #     '''
    #     if self.parent is not None and new_parent is not None:  #assigned a new parent
    #         self.parent.children.remove(self)
    #         self.parent = new_parent
    #         self.parent.children.add(self)
    #     #calculate new cost from root #FIXME: redundancy
    #     assert(self.parent.reachable_set.contains(self.state))
    #     cost_self_from_parent, path_self_from_parent = self.parent.reachable_set.plan_collision_free_path_in_set(self.state)
    #     cost_root_to_parent = self.parent.cost_from_root
    #     self.cost_from_parent = cost_self_from_parent
    #     self.cost_from_root = cost_root_to_parent+self.cost_from_parent
    #     self.path_from_parent = path_self_from_parent
    #     #calculate new cost for children
    #     for child in self.children:
    #         child.update_parent()
    #     # print(self.parent.state, 'path', self.path_from_parent)
    #     # assert(np.all(self.parent.state==self.path_from_parent[0]))
    #     # assert(np.all(self.state==self.path_from_parent[1]))

class ReachableSetTree:
    '''
    Wrapper for a fast data structure that can help querying
    '''
    def __init__(self):
        pass

    def insert(self, id, reachable_set):
        raise('NotImplementedError')

    def nearest_k_neighbor_ids(self, query_state, k=1):
        raise('NotImplementedError')

    def d_neighbor_ids(self, query_state, d = np.inf):
        '''
        For rewiring only
        :param query_state:
        :param d:
        :return:
        '''
        raise('NotImplementedError')

class StateTree:
    '''
    Wrapper for a fast data structure that can help querying
    '''
    def __init__(self):
        pass

    def insert(self, id, state):
        raise('NotImplementedError')

    def state_ids_in_reachable_set(self, query_reachable_set):
        '''
        For rewiring only
        :param query_reachable_set:
        :return:
        '''
        raise('NotImplementedError')

class R3T:
    def __init__(self, root_state, compute_reachable_set, sampler, reachable_set_tree, state_tree, path_class, rewire_radius = None):
        '''
        Base RG-RRT*
        :param root_state: The root state
        :param compute_reachable_set: A function that, given a state, returns its reachable set
        :param sampler: A function that randomly samples the state space
        :param reachable_set_tree: A StateTree object for fast querying
        :param path_class: A class handel that is used to represent path
        '''
        self.root_node = Node(root_state, compute_reachable_set(root_state), np.asarray([root_state,root_state]),cost_from_parent=0)
        self.root_id = hash(str(root_state))
        self.state_dim = root_state[0]
        self.compute_reachable_set = compute_reachable_set
        self.sampler = sampler
        self.goal_state = None
        self.goal_node = None
        self.path_class = path_class
        self.state_tree = state_tree()
        self.state_tree.insert(self.root_id,self.root_node.state)
        self.reachable_set_tree = reachable_set_tree() #tree for fast node querying
        self.reachable_set_tree.insert(self.root_id, self.root_node.reachable_set)
        self.state_to_node_map = dict()
        self.state_to_node_map[self.root_id] = self.root_node
        self.node_tally = 0
        self.rewire_radius=rewire_radius

    def create_child_node(self, parent_node, child_state, true_dynamics_path, cost_from_parent = None, path_from_parent = None):
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
        # assert (parent_node.reachable_set.contains(child_state))
        cost_from_parent, path_from_parent = parent_node.reachable_set.plan_collision_free_path_in_set(child_state)
        # construct a new node
        new_node = Node(child_state, self.compute_reachable_set(child_state), true_dynamics_path,
                        parent=parent_node, path_from_parent=path_from_parent, cost_from_parent=cost_from_parent)
        parent_node.children.add(new_node)
        return new_node

    def extend(self, new_state, nearest_node, true_dynamics_path, explore_deterministic_next_state=True):
        """

        :param new_state:
        :param nearest_node:
        :param true_dynamics_path:
        :param explore_deterministic_next_state:
        :return: is_extended, new_node, deterministic_next_state
        """
        # check for obstacles
        if explore_deterministic_next_state:
            cost_to_go, path, deterministic_next_state = nearest_node.reachable_set.plan_collision_free_path_in_set(new_state,
                                                                                          return_deterministic_next_state=explore_deterministic_next_state)
        else:
            cost_to_go, path = nearest_node.reachable_set.plan_collision_free_path_in_set(new_state, return_deterministic_next_state=explore_deterministic_next_state)
        #FIXME: Support for partial extensions
        if path is None:
            if explore_deterministic_next_state:
                return False, None, None
            else:
                return False, None
        new_node = self.create_child_node(nearest_node, new_state, cost_from_parent=cost_to_go, path_from_parent=path, true_dynamics_path=true_dynamics_path)
        if explore_deterministic_next_state:
            return True, new_node, deterministic_next_state
        else:
            return True, new_node

    def build_tree_to_goal_state(self, goal_state, allocated_time = 20, stop_on_first_reach = False, rewire=False, explore_deterministic_next_state = True, max_nodes_to_add = int(1e3),\
                                 save_true_dynamics_path = False):
        '''
        Builds a RG-RRT* Tree to solve for the path to a goal.
        :param goal_state:  The goal for the planner.
        :param allocated_time: Time allowed (in seconds) for the planner to run. If time runs out before the planner finds a path, the code will be terminated.
        :param stop_on_first_reach: Whether the planner should continue improving on the solution if it finds a path to goal before time runs out.
        :param rewire: Whether to do RRT*
        :param explore_deterministic_next_state: perform depth-first exploration (no sampling, just build node tree) when the reachable set is a point
        :return: The goal node as a Node object. If no path is found, None is returned. self.goal_node is set to the return value after running.
        '''
        #TODO: Timeout and other termination functionalities
        start = default_timer()
        self.goal_state = goal_state
        # For cases where root node can lead directly to goal
        contains_goal, true_dynamics_path = self.root_node.reachable_set.contains_goal(self.goal_state)
        if contains_goal:
            # check for obstacles
            # cost_to_go, path = new_node.reachable_set.plan_collision_free_path_in_set(goal_state)
            # allow for fuzzy goal check
            # if cost_to_go == np.inf:
            #     continue
            # if cost_to_go != np.inf:  #allow for fuzzy goal check
                # add the goal node to the tree
            goal_node = self.create_child_node(self.root_node, goal_state, true_dynamics_path)
            if rewire:
                self.rewire(goal_node)
            self.goal_node=goal_node

        while True:
            if stop_on_first_reach:
                if self.goal_node is not None:
                    print('Found path to goal with cost %f in %f seconds after exploring %d nodes' % (self.goal_node.cost_from_root,
                    default_timer() - start, self.node_tally))
                    return self.goal_node
            if default_timer()-start>allocated_time:
                if self.goal_node is None:
                    print('Unable to find path within %f seconds!' % (default_timer() - start))
                    return None
                else:
                    print('Found path to goal with cost %f in %f seconds after exploring %d nodes' % (self.goal_node.cost_from_root,
                    default_timer() - start, self.node_tally))
                    return self.goal_node

            #sample the state space
            sample_is_valid = False
            sample_count = 0
            while not sample_is_valid:
                random_sample = self.sampler()
                sample_count+=1
                # map the states to nodes
                try:
                    nearest_state_id_list = list(self.reachable_set_tree.nearest_k_neighbor_ids(random_sample, k=1))  # FIXME: necessary to cast to list?
                    discard = True
                    nearest_node = self.state_to_node_map[nearest_state_id_list[0]]
                    # find the closest state in the reachable set and use it to extend the tree
                    new_state, discard, true_dynamics_path = nearest_node.reachable_set.find_closest_state(random_sample, save_true_dynamics_path=save_true_dynamics_path)
                    new_state_id = hash(str(new_state))
                    # add the new node to the set tree if the new node is not already in the tree
                    if new_state_id in self.state_to_node_map or discard:
                        # FIXME: how to prevent repeated state exploration?
                        # print('Warning: state already explored')
                        continue  # #sanity check to prevent numerical errors

                    if not explore_deterministic_next_state:
                        is_extended, new_node = self.extend(new_state, nearest_node, true_dynamics_path, explore_deterministic_next_state=False)
                    else:
                        is_extended, new_node, deterministic_next_state = self.extend(new_state, nearest_node, true_dynamics_path, explore_deterministic_next_state=True)
                except Exception as e:
                    # print('Caught %s' % e)
                    is_extended = False
                if not is_extended:
                    # print('Extension failed')
                    continue
                else:
                    sample_is_valid = True
                #FIXME: potential infinite loop
            if sample_count>100:
                print('Warning: sample count %d' %sample_count)
            if not explore_deterministic_next_state:
                self.reachable_set_tree.insert(new_state_id, new_node.reachable_set)
                self.state_tree.insert(new_state_id, new_node.state)
                try:
                    assert(new_state_id not in self.state_to_node_map)
                except:
                    print('State id hash collision!')
                    print('Original state is ', self.state_to_node_map[new_state_id].state)
                    print('Attempting to insert', new_node.state)
                    raise AssertionError
                self.state_to_node_map[new_state_id] = new_node
                self.node_tally = len(self.state_to_node_map)
                #rewire the tree
                if rewire:
                    self.rewire(new_node)
                #In "find path" mode, if the goal is in the reachable set, we are done
                contains_goal, true_dynamics_path = new_node.reachable_set.contains_goal(self.goal_state)
                if contains_goal:
                    # check for obstacles
                    # add the goal node to the tree
                    # cost_to_go, path = new_node.reachable_set.plan_collision_free_path_in_set(goal_state)
                    # allow for fuzzy goal check
                    # if cost_to_go == np.inf:
                    #     continue
                    goal_node = self.create_child_node(new_node, self.goal_state, true_dynamics_path)
                    diff = np.subtract(self.goal_state, true_dynamics_path)
                    diff_norm = np.linalg.norm(diff, axis=1)
                    if rewire:
                        self.rewire(goal_node)
                    self.goal_node=goal_node
            else:
                nodes_to_add = [new_node]
                for iteration_count in range(int(max_nodes_to_add)):
                    # No longer deterministic
                    if new_node.reachable_set.deterministic_next_state is None:
                        break
                    # Already added
                    if hash(str(new_node.reachable_set.deterministic_next_state[-1])) in self.state_to_node_map:
                        break
                    try:
                        if save_true_dynamics_path:
                            is_extended, new_node,deterministic_next_state = self.extend(new_node.reachable_set.deterministic_next_state[-1], \
                                                                                         new_node, true_dynamics_path=new_node.reachable_set.deterministic_next_state,explore_deterministic_next_state=True)
                        else:
                            is_extended, new_node, deterministic_next_state = self.extend(
                                new_node.reachable_set.deterministic_next_state[-1],
                                new_node, true_dynamics_path=[new_node.state,new_node.reachable_set.deterministic_next_state[-1]],
                                explore_deterministic_next_state=True)
                    except Exception as e:
                        # print('Caught %s' %e)
                        is_extended=False
                    if not is_extended:  # extension failed
                        break
                    nodes_to_add.append(new_node)
                if iteration_count == max_nodes_to_add-1:
                    print('Warning: hit max_nodes_to_add')
                for new_node in nodes_to_add:
                    new_state_id = hash(str(new_node.state))
                    try:
                        assert(new_state_id not in self.state_to_node_map)
                    except:
                        print('State id hash collision!')
                        print('Original state is ', self.state_to_node_map[new_state_id].state)
                        print('Attempting to insert', new_node.state)
                        raise AssertionError
                    self.reachable_set_tree.insert(new_state_id, new_node.reachable_set)
                    self.state_tree.insert(new_state_id, new_node.state)
                    self.state_to_node_map[new_state_id] = new_node
                    self.node_tally = len(self.state_to_node_map)
                    #rewire the tree
                    if rewire:
                        self.rewire(new_node)
                    #In "find path" mode, if the goal is in the reachable set, we are done
                    contains_goal, true_dynamics_path = new_node.reachable_set.contains_goal(self.goal_state)
                    if contains_goal:
                    # check for obstacles
                        cost_to_go, path = new_node.reachable_set.plan_collision_free_path_in_set(goal_state)
                        #allow for fuzzy goal check
                        # if cost_to_go == np.inf:
                        #     continue
                        # add the goal node to the tree
                        goal_node = self.create_child_node(new_node, self.goal_state, true_dynamics_path)
                        if rewire:
                            self.rewire(goal_node)
                        self.goal_node=goal_node

    def rewire(self, new_node):
        # rewire_parent_candidate_states = list(self.reachable_set_tree.d_neighbor_ids(new_node.state))
        # #rewire the parent of the new node
        # best_new_parent = None
        # best_cost_to_new_node = new_node.cost_from_root
        # for cand_state in rewire_parent_candidate_states:
        #     parent_candidate_node = self.state_to_node_map[cand_state]
        #     if parent_candidate_node==new_node.parent or parent_candidate_node==new_node:
        #         continue
        #     #check if it's better to connect to new node through the candidate
        #     if not parent_candidate_node.reachable_set.contains(new_node.state):
        #         continue
        #     cost_to_go, path = parent_candidate_node.reachable_set.plan_collision_free_path_in_set(new_node.state)
        #     if parent_candidate_node.cost_from_root+cost_to_go < best_cost_to_new_node:
        #         best_new_parent = parent_candidate_node
        #         best_cost_to_new_node = parent_candidate_node.cost_from_root+cost_to_go
        #
        # #update if the best parent is changed
        # if best_new_parent is not None:
        #     new_node.update_parent(best_new_parent)

        cand_ids = self.state_tree.state_ids_in_reachable_set(new_node.reachable_set)
        #try to make the new node the candidate's parent
        for cand_id in cand_ids:
            candidate_node = self.state_to_node_map[cand_id]
            if candidate_node == new_node.parent or candidate_node == self.root_node:
                continue
            if not new_node.reachable_set.contains(candidate_node.state):
                continue
            cost_to_go, path = new_node.reachable_set.plan_collision_free_path_in_set(candidate_node.state)
            if candidate_node.cost_from_root > cost_to_go+new_node.cost_from_root:
                print('rewired!')
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
