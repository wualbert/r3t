from box_world.box_world_rg_rrt_star import *

def test_simple_empty_world():
    root = np.asarray([1,0.5])
    goal = np.asarray([9,9])
    world_bound = AABB([(0,0),(10,10)])
    world = BoxWorldMap(world_bound)
    rrt = BW_RGRRTStar(root,world)
    goal_node = rrt.build_tree_to_goal_state(goal)
    print(rrt.root_node)
    print('goal', goal_node)
    rrt.visualize()

if __name__ == '__main__':
    test_simple_empty_world()
