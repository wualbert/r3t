from dubin_car.dubin_car_rg_rrt_star import *
from dubin_car.visualize.visualize_dc_rg_rrt_star import *
import dubin_car.base_reachable_set
from timeit import default_timer

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
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf, explore_deterministic_next_state = False)
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
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=False, allocated_time=time, explore_deterministic_next_state = False)
    if goal_node is None:
        print('No path found!')
        return
    visualize_tree(rrt, world)

def test_fixed_medium_boxy_world_optimality(try_duration,tries = 5, plot_routes = False):
    root = np.asarray([0,0,np.pi/2])
    goal = np.asarray([26,24,np.pi/5])
    world_bound = AABB([(-10,-10),(30,30)])
    world = DC_Map(world_bound)
    box1=AABB([(-1,10),(9,15)])
    box2=AABB([(11,7),(24,11)])
    box3 = AABB([(17, 17), (21,24)])
    box4 = AABB([(5, -3), (10, 7)])
    obstacles = [box1, box2, box3, box4]
    for obs in obstacles:
        world.add_obstacle(obs)
    rrt = DC_RGRRTStar(root,world.point_collides_with_obstacle, world.random_sampler, rewire_radius=5)

    costs = np.zeros(tries)
    nodes_explored = np.zeros(tries)
    times = np.zeros(tries)

    #first try
    start_time = default_timer()
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, explore_deterministic_next_state = False)
    total_duration = default_timer()-start_time
    costs[0] = goal_node.cost_from_root
    times[0] = total_duration
    nodes_explored[0] = rrt.node_tally
    if plot_routes:
        fig, ax = visualize_tree(rrt, world,visualize_all_paths=True)
        ax.set_title('First solution (found in %.3f seconds)' % total_duration)
        plt.savefig("first_sol.png", dpi=150)
        plt.close()

    for i in range(1,tries):
        start_time = default_timer()
        goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=False,allocated_time=try_duration, explore_deterministic_next_state = False)
        try_duration = default_timer()-start_time
        total_duration+=try_duration
        costs[i] = goal_node.cost_from_root
        times[i] = total_duration
        nodes_explored[i] = rrt.node_tally
        if plot_routes:
            fig, ax = visualize_tree(rrt, world,visualize_all_paths=True)
            ax.set_title('Solution after %.3f seconds' % total_duration)
            plt.savefig("try_"+str(i)+".png", dpi=150)
            plt.close()
    fig, ax = plt.subplots()

    ax.plot(times,costs)
    ax.set_xlabel('Time(s)')
    ax.set_ylabel('Cost to goal')
    ax.set_title('Cost vs. Time')
    plt.savefig("cost_vs_time.png", dpi=150)
    plt.close()
    fig, ax = plt.subplots()
    ax.plot(times,nodes_explored)
    ax.set_xlabel('Time(s)')
    ax.set_ylabel('Nodes explored')
    ax.set_title('Nodes explored vs. Time')
    plt.savefig("nodes_explored_vs_time.png", dpi=150)

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
    start_time = default_timer()
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=True, allocated_time=np.inf, explore_deterministic_next_state = False)
    if goal_node is None:
        print('No path found!')
        return
    fig, ax =  visualize_tree(rrt, world)
    ax.set_title('Solution after %.3f seconds' % (default_timer()-start_time))
    plt.savefig("first_sol.png", dpi=150)
    goal_node = rrt.build_tree_to_goal_state(goal,stop_on_first_reach=False, allocated_time=50)
    fig, ax =  visualize_tree(rrt, world)
    ax.set_title('Solution after %.3f seconds' % (default_timer()-start_time))
    plt.savefig("plus_50.png", dpi=150)


if __name__ == '__main__':
    test_fixed_medium_boxy_world_optimality(try_duration=5, plot_routes=True)