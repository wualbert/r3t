from common.r3t import *
from bounding_box_closest_polytope.lib.box import AABB, point_in_box
from bounding_box_closest_polytope.visualization.visualize import visualize_boxes
from rtree import index
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque

class BW_Path(Path):
    def __init__(self, start, end):
        self.waypoints = deque()
        self.waypoints.append(start)
        self.waypoints.append(end)
        # print(self.waypoints)
    def __repr__(self):
        return('Line segment from '+ str(self.waypoints[0]) + ' to '+ str(self.waypoints[1]))
    def append(self, path):
        self.waypoints.extend(path.waypoints)

class BoxWorldMap:
    def __init__(self, world_bounding_box):
        self.obstacles = [] #pairs of coordinates ((x1,y1),(x2,y2)
        self.world_bounding_box = world_bounding_box
        self.collision_memo = {}
    def random_sampler(self):
        x_rand = np.random.uniform(self.world_bounding_box.u[0], self.world_bounding_box.v[0])
        y_rand = np.random.uniform(self.world_bounding_box.u[1], self.world_bounding_box.v[1])
        return np.asarray([x_rand,y_rand])

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def compute_reachable_set(self, state):
        return BW_ReachabelSet(state, self)

    def point_collides_with_obstacle(self, point):
        for obs in self.obstacles:
            # print(point)
            if point_in_box(point, obs):
                return True
        return False

    def line_collides_with_obstacle(self, start, goal):
        string_rep = (str(start),str(goal))
        if string_rep in self.collision_memo:
            return self.collision_memo[string_rep]
        sample_count = 100 #FIXME
        x_samples = np.linspace(start[0], goal[0], sample_count)
        y_samples = np.linspace(start[1], goal[1], sample_count)
        for i in range(sample_count):
            if self.point_collides_with_obstacle(np.asarray([x_samples[i], y_samples[i]])):
                self.collision_memo[string_rep]=True
                return True
        self.collision_memo[string_rep] = False
        # print(start,goal)
        return False

    def cost_to_go(self, start, goal):
        if self.line_collides_with_obstacle(start,goal):
            return np.inf
        else:
            return np.linalg.norm(goal-start)

class BW_ReachabelSet(ReachableSet):
    def __init__(self, state, world_map):
        ReachableSet.__init__(self, BW_Path)
        self.state = state
        self.reachable_range = 3 #arbitrary parameter
        self.world_map = world_map

    def contains(self, goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        if np.linalg.norm(goal_state-self.state)<=self.reachable_range:
            return True

    def plan_collision_free_path_in_set(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). Cost to go is set to infinity
        '''
        if np.linalg.norm(goal_state-self.state)>self.reachable_range:
            return (np.inf, None)
        else:
            if self.world_map.line_collides_with_obstacle(self.state, goal_state):
                return (np.inf, None)
            else:
                return self.world_map.cost_to_go(self.state, goal_state), BW_Path(self.state, goal_state)

    def find_closest_state(self, query_point):
        '''
        Finds the closest point in the reachable set to the query point
        :param query_point:
        :return: Tuple (closest_point, closest_point_is_self.state)
        '''
        if self.contains(query_point):
            return query_point, False #FIXME: what if the closest point is self.state?
        else:
            unit_direction = (query_point-self.state)/np.linalg.norm(query_point-self.state)
            return self.state+unit_direction*self.reachable_range

class BW_StateTree(ReachableSetTree):
    def __init__(self):
        ReachableSetTree.__init__(self)
        self.tree = index.Index()

    def insert(self, id, state):
        self.tree.insert(id, [state[0],state[1],state[0],state[1]])

    def nearest_k_neighbor_ids(self, query_state, k=1):
        return self.tree.nearest([query_state[0],query_state[1],query_state[0],query_state[1]], k)

    def d_neighbor_ids(self, query_state, d = np.inf):
        return self.tree.intersection([query_state[0]-d/2,query_state[1]-d/2,query_state[0]+d/2,query_state[1]+d/2], objects=False)


class BW_R3T(R3T):
    def __init__(self, root_state, world_map,rewire_radius):
        '''
        Base RG-RRT*
        :param root_state: The root state
        :param compute_reachable_set: A function that, given a state, returns its reachable set
        :param sampler: A function that randomly samples the state space
        :param state_tree: A StateTree object for fast querying
        '''
        state_tree = BW_StateTree()
        self.world_map = world_map
        R3T.__init__(self, root_state, self.world_map.compute_reachable_set,
                     self.world_map.random_sampler, state_tree, BW_Path, rewire_radius)

    def visualize(self, visualize_nodes=True, visualize_edges=True, visualize_answer=True,visualize_obstacles=True,ax = None, fig=None):
        if ax is None or fig is None:
            fig, ax = plt.subplots()
        #visualize start
        ax.scatter([self.root_node.state[0]],[self.root_node.state[1]],facecolor='r')
        #visualize goal
        if self.goal_state is not None:
            ax.scatter([self.goal_state[0]], [self.goal_state[1]], facecolor='g')
        #visualize nodes and edges
        node_queue =deque()
        node_queue.append(self.root_node)
        # print('root', len(self.root_node.children))
        if visualize_obstacles:
            visualize_boxes(self.world_map.obstacles, ax=ax, fig=fig, fill=True, alpha = 0.3)

        if visualize_nodes or visualize_edges:
            tree_path_segments = []
            while(len(node_queue)>0):
                #pop from the top of the queue
                node = node_queue.popleft()
                #get the children of this node and add to the queue
                children_nodes = node.children
                node_queue.extend(children_nodes)
                #visualize the state
                if visualize_nodes:
                    ax.scatter([node.state[0]], [node.state[1]], facecolor='k', s=.2)
                if visualize_edges and node.parent is not None:
                    tree_path_segments.append(node.path_from_parent.waypoints)
                    # segment = [[node.path_from_parent[0][0],node.path_from_parent[1][0]],
                    #            [node.path_from_parent[0][1],node.path_from_parent[1][1]]]
            if visualize_edges:
                tree_lc = mc.LineCollection(tree_path_segments, colors='k', linewidths=0.2, alpha=0.5)
                ax.add_collection(tree_lc)
        if visualize_answer:
            node = self.goal_node
            goal_path_segments = []
            while node.parent!=None:
                goal_path_segments.append(node.path_from_parent.waypoints)
                ax.scatter([node.state[0]], [node.state[1]], facecolor='b', s=2)
                node = node.parent
            goal_lc = mc.LineCollection(goal_path_segments, colors='b', linewidths=1)
            ax.add_collection(goal_lc)
        plt.xlim(self.world_map.world_bounding_box.u[0],self.world_map.world_bounding_box.v[0])
        plt.ylim(self.world_map.world_bounding_box.u[1],self.world_map.world_bounding_box.v[1])
        return fig, ax