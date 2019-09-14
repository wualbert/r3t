import numpy as np
from rtree import index

from timeit import default_timer


class Node:
    def __init__(self, state, parent = None, path_from_parent = None,
                 children = None, cost_from_parent = np.inf, true_dynamics_path=None):
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
        self.parent = parent
        self.path_from_parent = path_from_parent
        self.cost_from_parent = cost_from_parent
        self.true_dynamics_path=true_dynamics_path
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

class StateTree:
    def __init__(self):
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

    def find_nearest(self, state):
        nearest_box = np.hstack([state, state])
        state_id = list(self.state_idx.nearest(nearest_box,1))[0]
        # print('nearest',self.state_id_to_state[state_id])
        return self.state_id_to_state[state_id]

global dropped_states
class BasicRRT:
    def __init__(self, root_state, sampler, reached_goal_function, plan_collision_free_path_towards, state_tree=StateTree(), rewire_radius = None):
        self.root_node = Node(root_state, cost_from_parent=0)
        self.root_id = hash(str(root_state))
        self.state_dim = root_state[0]
        self.sampler = sampler
        self.reached_goal = reached_goal_function
        self.goal_state = None
        self.goal_node = None
        self.state_tree = state_tree
        self.state_tree.insert(self.root_id,self.root_node.state)
        self.state_to_node_map = dict()
        self.state_to_node_map[self.root_id] = self.root_node
        self.node_tally = 0
        self.rewire_radius=rewire_radius
        self.plan_collision_free_path_towards=plan_collision_free_path_towards
        global dropped_states
        dropped_states = 0
    def create_child_node(self, parent_node, child_state, cost_from_parent, path_from_parent, true_dynamics_path=None):
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
        # construct a new node
        new_node = Node(child_state, parent=parent_node, path_from_parent=path_from_parent, cost_from_parent=cost_from_parent)
        parent_node.children.add(new_node)
        return new_node

    def extend(self, new_state, nearest_node, explore_deterministic_next_state=False):
        # test for possibility to extend
        cost_to_go, end_state = self.plan_collision_free_path_towards(nearest_node.state, new_state)
        if end_state is None:
            return False, None
        new_node = self.create_child_node(nearest_node, end_state, cost_to_go, end_state)
        return True, new_node

    def build_tree_to_goal_state(self, goal_state, allocated_time=20, stop_on_first_reach=False, rewire=False,
                                     explore_deterministic_next_state=True, max_nodes_to_add=int(1e9)):
        global dropped_states
        start = default_timer()
        self.goal_state = goal_state
        # For cases where root node can lead directly to goal
        if self.reached_goal(self.root_node.state, goal_state):
            goal_node = self.create_child_node(self.root_node, goal_state)
            # if rewire:
            #     self.rewire(goal_node)
            self.goal_node=goal_node

        while True:
            if stop_on_first_reach:
                if self.goal_node is not None:
                    print('Found path to goal with cost %f in %f seconds after exploring %d nodes' % (self.goal_node.cost_from_root,
                    default_timer() - start, self.node_tally))
                    return self.goal_node
            if default_timer()-start>allocated_time:
                print('Dropped %i states' %dropped_states)
                if self.goal_node is None:
                    print('Unable to find path within %f seconds!' % (default_timer() - start))
                    return None
                else:
                    print('Found path to goal with cost %f in %f seconds after exploring %d nodes' % (self.goal_node.cost_from_root,
                    default_timer() - start, self.node_tally))
                    return self.goal_node

            #sample the state space
            state_sample = self.sampler()
            # if not explore_deterministic_next_state:
            nearest_state = self.state_tree.find_nearest(state_sample)
            try:
                nearest_node = self.state_to_node_map[hash(str(nearest_state))]
            except:
                print('node not found!')
                continue
            is_extended, new_node = self.extend(state_sample, nearest_node)
            if not is_extended: #extension failed
                print('Warning: extension failed')
                dropped_states += 1
                continue
            new_state_id = hash(str(new_node.state))
            if new_state_id in self.state_to_node_map:
                continue
            # try:
            #     assert(new_state_id not in self.state_to_node_map)
            # except:
            #     print('State id hash collision!')
            #     print('Original state is ', self.state_to_node_map[new_state_id].state)
            #     print('Attempting to insert', new_node.state)
            #     raise AssertionError
            self.state_tree.insert(new_state_id, new_node.state)
            self.state_to_node_map[new_state_id] = new_node
            #
            # print('snm', len(self.state_to_node_map))
            # print(len(self.state_tree.state_id_to_state))
            self.node_tally = len(self.state_to_node_map)
            # TODO
            #rewire the tree
            # if rewire:
            #     self.rewire(new_node)
            #In "find path" mode, if the goal is in the reachable set, we are done
            if self.reached_goal(new_node.state, goal_state): #FIXME: support for goal region
                # add the goal node to the tree
                is_extended, goal_node = self.extend(goal_state, new_node)
                # TODO
                # if rewire:
                #     self.rewire(goal_node)
                if is_extended:
                    self.goal_node=goal_node
            # TODO
            # else:
            #     is_extended, new_node = self.extend(state_sample, nearest_node, True)
            #     if not is_extended:  # extension failed
            #         print('Warning: extension failed')
            #         continue
            #     nodes_to_add = [new_node]
            #     iteration_count=0
            #     while iteration_count<max_nodes_to_add:
            #         # No longer deterministic
            #         if new_node.reachable_set.deterministic_next_state is None:
            #             break
            #         # Already added
            #         if hash(str(new_node.reachable_set.deterministic_next_state)) in self.state_to_node_map:
            #             break
            #         is_extended, new_node = self.extend(new_node.reachable_set.deterministic_next_state, new_node)
            #         if not is_extended:  # extension failed
            #             break
            #         nodes_to_add.append(new_node)
            #         iteration_count+=1
            #     # print('%d nodes to add' %len(nodes_to_add))
            #     for new_node in nodes_to_add:
            #         new_state_id = hash(str(new_node.state))
            #         self.reachable_set_tree.insert(new_state_id, new_node.reachable_set)
            #         self.state_tree.insert(new_state_id, new_node.state)
            #         try:
            #             assert(new_state_id not in self.state_to_node_map)
            #         except:
            #             print('State id hash collision!')
            #             print('Original state is ', self.state_to_node_map[new_state_id].state)
            #             print('Attempting to insert', new_node.state)
            #             raise AssertionError
            #         self.state_to_node_map[new_state_id] = new_node
            #         self.node_tally = len(self.state_to_node_map)
            #         #rewire the tree
            #         if rewire:
            #             self.rewire(new_node)
            #         #In "find path" mode, if the goal is in the reachable set, we are done
            #         if new_node.reachable_set.contains_goal(goal_state): #FIXME: support for goal region
            #             # check for obstacles
            #             cost_to_go, path = new_node.reachable_set.plan_collision_free_path_in_set(goal_state)
            #             #allow for fuzzy goal check
            #             # if cost_to_go == np.inf:
            #             #     continue
            #             # add the goal node to the tree
            #             goal_node = self.create_child_node(new_node, goal_state)
            #             if rewire:
            #                 self.rewire(goal_node)
            #             self.goal_node=goal_node

class RGRRT(BasicRRT):
    def __init__(self, root_state, sampler, reached_goal_function, plan_collision_free_path_towards, state_tree=StateTree(), rewire_radius = None):
        self.root_node = Node(root_state, cost_from_parent=0, true_dynamics_path=[root_state, root_state])
        self.root_id = hash(str(root_state))
        self.state_dim = root_state[0]
        self.sampler = sampler
        self.reached_goal = reached_goal_function
        self.goal_state = None
        self.goal_node = None
        self.state_tree = state_tree
        self.state_tree.insert(self.root_id,self.root_node.state)
        self.state_to_node_map = dict()
        self.true_path_map = dict()
        self.state_to_node_map[self.root_id] = self.root_node
        self.node_tally = 0
        self.rewire_radius=rewire_radius
        self.plan_collision_free_path_towards=plan_collision_free_path_towards
        self.reachable_set_tree=StateTree()
        self.reachabe_state_to_node_map=dict()
        reachable_states, true_dynamics_path = self.get_reachable_states(root_state)
        for i, rs in enumerate(reachable_states):
            rs_hash = hash(str(rs))
            self.reachable_set_tree.insert(rs_hash, rs)
            self.reachabe_state_to_node_map[rs_hash]=self.root_node
            self.true_path_map[str(root_state)+str(rs)]=true_dynamics_path[i]


    def create_child_node(self, parent_node, child_state, cost_from_parent, path_from_parent, true_dynamics_path):
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
        # construct a new node
        new_node = Node(child_state, parent=parent_node, path_from_parent=path_from_parent, cost_from_parent=cost_from_parent, \
                        true_dynamics_path=true_dynamics_path)
        parent_node.children.add(new_node)
        reachable_states, true_dynamics_path = self.get_reachable_states(child_state)
        for i, rs in enumerate(reachable_states):
            rs_hash = hash(str(rs))
            self.reachable_set_tree.insert(rs_hash, rs)
            self.reachabe_state_to_node_map[rs_hash]=new_node
            self.true_path_map[str(child_state)+str(rs)]=true_dynamics_path[i]
        return new_node

    def get_reachable_states(self, state):
        raise NotImplementedError

    def extend(self, new_state, nearest_node, explore_deterministic_next_state=False):
        # test for possibility to extend
        cost_to_go, end_state, best_states_list = self.plan_collision_free_path_towards(nearest_node.state, new_state)
        if end_state is None:
            return False, None
        new_node = self.create_child_node(nearest_node, end_state, cost_to_go, end_state, true_dynamics_path=best_states_list)
        return True, new_node

    def force_extend(self, new_state, nearest_node, true_dynamics_path):
        # test for possibility to extend
        new_node = self.create_child_node(nearest_node, new_state, np.linalg.norm(new_state-nearest_node.state), new_state,true_dynamics_path)
        return True, new_node

    def build_tree_to_goal_state(self, goal_state, allocated_time=20, stop_on_first_reach=False, rewire=False,
                                     explore_deterministic_next_state=True, max_nodes_to_add=int(1e9)):
        start = default_timer()
        self.goal_state = goal_state
        # For cases where root node can lead directly to goal
        if self.reached_goal(self.root_node.state, goal_state):
            goal_node = self.create_child_node(self.root_node, goal_state)
            # if rewire:
            #     self.rewire(goal_node)
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
            state_sample = self.sampler()
            nearest_reachable_state = self.reachable_set_tree.find_nearest(state_sample)
            nearest_node = self.reachabe_state_to_node_map[hash(str(nearest_reachable_state))]
            new_state_id = hash(str(nearest_reachable_state))
            if new_state_id in self.state_to_node_map:
                continue
            true_dynamics_path = self.true_path_map[str(nearest_node.state)+str(nearest_reachable_state)]
            is_extended, new_node = self.force_extend(nearest_reachable_state, nearest_node, true_dynamics_path)
            if not is_extended: #extension failed
                print('Warning: extension failed')
                continue
            # try:
            #     assert(new_state_id not in self.state_to_node_map)
            # except:
            #     print('State id hash collision!')
            #     print('Original state is ', self.state_to_node_map[new_state_id].state)
            #     print('Attempting to insert', new_node.state)
            #     raise AssertionError
            self.state_tree.insert(new_state_id, new_node.state)
            self.state_to_node_map[new_state_id] = new_node

            # print('snm', len(self.state_to_node_map))
            # print(len(self.state_tree.state_id_to_state))
            self.node_tally = len(self.state_to_node_map)
            # TODO
            #rewire the tree
            # if rewire:
            #     self.rewire(new_node)
            #In "find path" mode, if the goal is in the reachable set, we are done
            if self.reached_goal(new_node.state, goal_state): #FIXME: support for goal region
                # add the goal node to the tree
                is_extended, goal_node = self.extend(goal_state, new_node)
                # TODO
                # if rewire:
                #     self.rewire(goal_node)
                if is_extended:
                    self.goal_node=goal_node
