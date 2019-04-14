from dubin_car.dubin_car_rg_rrt_star import *
from dubin_car.visualize.visualize_dc_rg_rrt_star import *
import dubin_car.base_reachable_set

def test_simple_empty_world():
    world_bound = AABB([(0, 0), (30, 30)])
    root = np.asarray([1,1,0])
    goal = np.asarray([29,29,np.pi])
    world = DC_Map(world_bound)
    rrt = DC_RGRRTStar(root,world.point_collides_with_obstacle, world.random_sampler, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)

def test_fixed_small_boxy_world():
    root = np.asarray([1,0.5,0])
    goal = np.asarray([9,9,np.pi/2])
    world_bound = AABB([(0,0),(10,10)])
    world = DC_Map(world_bound)
    box1=AABB([(3,4.5),(5,9)])
    box2=AABB([(7,2),(9,5)])
    box3 = AABB([(1, 1), (2, 3)])
    box4 = AABB([(3, 0.5), (6, 4)])
    box5 = AABB([(6, 6), (9, 8.5)])
    obstacles = [box1, box2,box3,box4,box5]
    for obs in obstacles:
        world.add_obstacle(obs)
    rrt = DC_RGRRTStar(root,world.point_collides_with_obstacle, world.random_sampler, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)

def test_fixed_small_boxy_world_long_time(time=15):
    root = np.asarray([1,0.5,0])
    goal = np.asarray([9,9,np.pi/2])
    world_bound = AABB([(0,0),(10,10)])
    world = DC_Map(world_bound)
    box1=AABB([(3,4.5),(5,9)])
    box2=AABB([(7,2),(9,5)])
    box3 = AABB([(1, 1), (2, 3)])
    box4 = AABB([(3, 0.5), (6, 4)])
    box5 = AABB([(6, 6), (9, 8.5)])
    obstacles = [box1, box2,box3,box4,box5]
    for obs in obstacles:
        world.add_obstacle(obs)
    rrt = DC_RGRRTStar(root,world.point_collides_with_obstacle, world.random_sampler, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=False, allocated_time=time)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)

def test_fixed_medium_boxy_world_hard(stop_on_first_reach,allocated_time):
    root = np.asarray([0,0,np.pi/2])
    goal = np.asarray([22,22,np.pi/4])
    world_bound = AABB([(-5,-5),(25,25)])
    world = DC_Map(world_bound)
    box1=AABB([(-1,10),(9,15)])
    box2=AABB([(11,7),(24,12)])
    # box3 = AABB([(1, 4), (4, 17)])
    box4 = AABB([(2, 1), (10, 8)])
    obstacles = [box1, box2, box4]
    for obs in obstacles:
        world.add_obstacle(obs)
    rrt = DC_RGRRTStar(root,world.point_collides_with_obstacle, world.random_sampler, rewire_radius=5)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=stop_on_first_reach,allocated_time=allocated_time)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)
    return goal_node.cost_from_root

def test_fixed_ring_boxy_world():
    root = np.asarray([0,-37,0])
    goal = np.asarray([0,37,np.pi*3/4])
    world_bound = AABB([(-50,-50),(50,50)])
    world = DC_Map(world_bound)
    box1=AABB([(-35,-35),(35,35)])
    box2=AABB([(-50,-50),(-40,50)])
    box3 = AABB([(40, -50), (50, 50)])
    box4 = AABB([(-50, -50), (50, -40)])
    box5 = AABB([(-50,40),(50,50)])
    obstacles = [box1, box2,box3,box4,box5]
    for obs in obstacles:
        world.add_obstacle(obs)
    rrt = DC_RGRRTStar(root,world.point_collides_with_obstacle, world.random_sampler, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)

if __name__ == '__main__':
    test_fixed_medium_boxy_world_hard(True,np.inf)
    plt.show()