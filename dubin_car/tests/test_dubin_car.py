from dubin_car.dubin_car_rg_rrt_star import *
from dubin_car.visualize.visualize_dc_rg_rrt_star import *
import dubin_car.base_reachable_set

def test_simple_empty_world():
    world_bound = AABB([(0, 0), (30, 30)])
    root = np.asarray([1,1,0])
    goal = np.asarray([29,29,np.pi])
    world = DC_Map(world_bound)
    rrt = DC_RGRRTStar(root,world.compute_reachable_set, world.random_sampler, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)

def test_fixed_very_boxy_world():
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
    rrt = DC_RGRRTStar(root,world.compute_reachable_set, world.random_sampler, rewire_radius=4)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)

# def test_fixed_box_ring_world():
#     root = np.asarray([1,0.5])
#     goal = np.asarray([9,9])
#     world_bound = AABB([(0,0),(10,10)])
#     world = DC_Map(world_bound)
#     box1=AABB([(3,4.5),(5,9)])
#     box2=AABB([(7,2),(9,5)])
#     box3 = AABB([(1, 1), (2, 3)])
#     box4 = AABB([(3, 0.5), (6, 4)])
#     box5 = AABB([(6, 6), (9, 8.5)])
#     obstacles = [box1, box2,box3,box4,box5]
#     for obs in obstacles:
#         world.add_obstacle(obs)
#     rrt = DC_RGRRTStar(root,world.compute_reachable_set, world.random_sampler, rewire_radius=4)
#     goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf)
#     if goal_node is None:
#         print('No path found!')
#         return
#     visualize_tree(rrt, world)

if __name__ == '__main__':
    test_fixed_very_boxy_world()
    plt.show()