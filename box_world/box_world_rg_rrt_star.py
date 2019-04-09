from common.rg_rrt_star import *
from bounding_box_closest_polytope.lib.box import AABB, point_to_box_distance
from rtree import index
import matplotlib.pyplot as plt

class BoxWorldMap:
    def __init__(self, world_bounding_box):
        self.obstacles = [] #pairs of coordinates ((x1,y1),(x2,y2)
        self.world_bounding_box = world_bounding_box
    def random_sampler(self):
        x_rand = np.random.uniform(self.world_bounding_box.u[0], self.world_bounding_box.v[0])
        y_rand = np.random.uniform(self.world_bounding_box.u[1], self.world_bounding_box.v[1])
        return np.asarray([x_rand,y_rand])

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def compute_reachable_set(self, state):
        return BWReachabelSet(state, self)

    def point_collides_with_obstacle(self, point):
        for obs in self.obstacles:
            if point_to_box_distance(point, obs) == 0:
                return True
        return False

    def line_collides_with_obstacle(self, start, goal):
        sample_count = 100 #FIXME
        x_samples = np.linspace(start[0], goal[0], sample_count)
        y_samples = np.linspace(start[0], goal[0], sample_count)
        for i in range(sample_count):
            if self.point_collides_with_obstacle(np.asarray([x_samples[i], y_samples[i]])):
                return True
        return False

    def cost_to_go(self, start, goal):
        if self.line_collides_with_obstacle(start,goal):
            return np.inf
        else:
            return np.linalg.norm(goal-start)

class BWReachabelSet(ReachableSet):
    def __init__(self, state, world_map):
        ReachableSet.__init__(self)
        self.state = state
        self.reachable_range = 5 #arbitrary
        self.world_map = world_map

    def contains(self, goal_state):
        '''
        Check if the state given is in this set
        :param state: query state
        :return: Boolean of whether the state is reachable
        '''
        if np.linalg.norm(goal_state-self.state)<=5:
            return True

    def plan_collision_free_path(self, goal_state):
        '''
        Plans a path between self.state and goal_state. Goals state must be in this reachable set
        :param state:
        :return: Tuple (cost_to_go, path). Cost to go is set to infinity
        '''

        if np.linalg.norm(goal_state-self.state)>5:
            return (np.inf, None)
        else:
            if self.world_map.line_collides_with_obstacle(self.state, goal_state):
                return (np.inf, None)
            else:
                return self.world_map.cost_to_go(self.state, goal_state), (self.state, goal_state)

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

class BW_StateTree(StateTree):
    def __init__(self):
        StateTree.__init__(self)
        self.tree = index.Index()

    def insert(self, id, state):
        self.tree.insert(id, [state[0],state[1],state[0],state[1]])

    def nearest_k_neighbors(self, query_state, k=1):
        return self.tree.nearest([query_state[0],query_state[1],query_state[0],query_state[1]], k)

    def d_neighbors(self, query_state, d = np.inf):
        return self.tree.intersection([query_state[0]-d/2,query_state[1]-d/2,query_state[0]+d/2,query_state[1]+d/2], objects=False)


class BW_RGRRTStar(RGRRTStar):
    def __init__(self, root_state, world_map):
        '''
        Base RG-RRT*
        :param root_state: The root state
        :param compute_reachable_set: A function that, given a state, returns its reachable set
        :param sampler: A function that randomly samples the state space
        :param state_tree: A StateTree object for fast querying
        '''
        state_tree = BW_StateTree()
        self.world_map = world_map
        RGRRTStar.__init__(self, root_state, self.world_map.compute_reachable_set,
                           self.world_map.random_sampler, state_tree)

    def visualize(self, visualize_nodes=True, visualize_edges=True, visualize_answer=True):
        fig, ax = plt.subplots()
        #visualize nodes
        ax.scatter([self.root_node.state[0]],[self.root_node.state[1]],facecolor='r')
        plt.xlim(self.world_map.world_bounding_box.u[0],self.world_map.world_bounding_box.v[0])
        plt.ylim(self.world_map.world_bounding_box.u[1],self.world_map.world_bounding_box.v[1])
        plt.show()