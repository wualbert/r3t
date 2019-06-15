import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer
from polytope_symbolic_system.examples.pendulum import Pendulum
from rg_rrt_star.continuous_system.continuous_system_rg_rrt_star import ContinuousSystem_RGRRTStar
from pypolycontain.visualization.visualize_2D import visualize_2D_zonotopes as visZ

def test_pendulum_planning():
    pendulum_system = Pendulum(input_limits=np.asarray([[-0.3],[0.3]]))
    goal_state = np.asarray([np.pi,0])
    def sampler():
        rnd = np.random.rand(2)
        rnd[0] = (rnd[0]-0.5)*3*np.pi
        rnd[1] = (rnd[1]-0.5)*1
        return rnd
    rrt = ContinuousSystem_RGRRTStar(pendulum_system, sampler, 1)
    rrt.build_tree_to_goal_state(goal_state,stop_on_first_reach=True, allocated_time=300)
    #get rrt polytopes

    polytope_reachable_sets = rrt.reachable_set_tree.polytope_reachable_sets.values()
    reachable_polytopes = []
    explored_states = []
    for prs in polytope_reachable_sets:
        reachable_polytopes.append(prs.polytope)
        explored_states.append(prs.parent_state)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    # fig, ax = visZ(reachable_polytopes, title="", alpha=0.07, fig=fig, ax=ax, color='gray')
    for explored_state in explored_states:
        plt.scatter(explored_state[0], explored_state[1], facecolor='red', s=6)
    plt.axes().set_aspect('equal')
    plt.xlabel('$x$')
    plt.ylabel('$\dot{x}$')
    plt.title('RRT Reachable Sets')
    # plt.savefig('RRT Reachable sets'+str(default_timer())+'.png', dpi=500)
    plt.show()



if __name__=='__main__':
    test_pendulum_planning()