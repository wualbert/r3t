from box_world.box_world_rg_rrt_star import *

def test_simple_empty_world():
    root = np.asarray([1,0.5])
    goal = np.asarray([9,9])
    world_bound = AABB([(0,0),(10,10)])
    world = BoxWorldMap(world_bound)
    rrt = BW_R3T(root, world, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal)
    # print(goal_node.parent.parent)
    ax, fig = rrt.visualize()
    # plt.show()

def test_fixed_very_boxy_world():
    root = np.asarray([1,0.5])
    goal = np.asarray([9,9])
    world_bound = AABB([(0,0),(10,10)])
    world = BoxWorldMap(world_bound)
    box1=AABB([(3,4.5),(5,9)])
    box2=AABB([(7,2),(9,5)])
    box3 = AABB([(1, 1), (2, 3)])
    box4 = AABB([(3, 0.5), (6, 4)])
    box5 = AABB([(6, 6), (9, 8.5)])
    obstacles = [box1, box2,box3,box4,box5]
    for obs in obstacles:
        world.add_obstacle(obs)
    rrt = BW_R3T(root, world, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal)
    # print(goal_node.parent.parent)
    ax, fig = rrt.visualize()
    plt.show()

if __name__ == '__main__':
    test_fixed_very_boxy_world()
