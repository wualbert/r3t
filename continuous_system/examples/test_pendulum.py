import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.pendulum import Pendulum
from rg_rrt_star.continuous_system.continuous_system_rg_rrt_star import ContinuousSystem_RGRRTStar
from pypolycontain.visualization.visualize_2D import visualize_2D_zonotopes as visZ
from closest_polytope.pypolycontain.lib.AH_polytope import distance_point
from utils.visualization import visualize_node_tree_2D

def test_pendulum_planning():
    initial_state = np.zeros(2)
    pendulum_system = Pendulum(initial_state= initial_state, input_limits=np.asarray([[-0.7],[0.7]]))
    goal_state = np.asarray([np.pi,0.0])
    def sampler():
        rnd = np.random.rand(2)
        rnd[0] = (rnd[0]-0.5)*2.5*np.pi
        rnd[1] = (rnd[1]-0.5)*20
        goal_bias = np.random.rand(1)
        if goal_bias<0.1:
            return goal_state
        return rnd

    def contains_goal_function(reachable_set, goal_state):
        distance, projection = distance_point(reachable_set.polytope, goal_state)
        if distance < 5e-2 and projection[1]<5e-2:
            return True
        return False

    rrt = ContinuousSystem_RGRRTStar(pendulum_system, sampler, 0.5, contains_goal_function=contains_goal_function)
    while(1):
        if rrt.build_tree_to_goal_state(goal_state,stop_on_first_reach=True, allocated_time=30) is not None:
            break
        #get rrt polytopes

        polytope_reachable_sets = rrt.reachable_set_tree.polytope_reachable_sets.values()
        reachable_polytopes = []
        explored_states = []
        for prs in polytope_reachable_sets:
            reachable_polytopes.append(prs.polytope)
            explored_states.append(prs.parent_state)
            # print(explored_states)
        print(len(explored_states))
        fig = plt.figure()
        ax = fig.add_subplot(111)
        fig, ax = visualize_node_tree_2D(rrt, fig, ax)
        # fig, ax = visZ(reachable_polytopes, title="", alpha=0.07, fig=fig, ax=ax, color='gray')
        # for explored_state in explored_states:
        #     plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
        ax.scatter(initial_state[0], initial_state[1], facecolor='red', s=10)
        ax.scatter(goal_state[0], goal_state[1], facecolor='green', s=10)
        ax.set_aspect('equal')
        plt.xlabel('$x$')
        plt.ylabel('$\dot{x}$')
        plt.title('RRT Reachable Sets')
        plt.savefig('RRT Reachable sets'+str(default_timer())+'.png', dpi=500)
        # plt.show()



if __name__=='__main__':
    test_pendulum_planning()