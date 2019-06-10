import numpy as np
from polytope_symbolic_system.examples.pendulum import Pendulum
from rg_rrt_star.continuous_system.continuous_system_rg_rrt_star import ContinuousSystem_RGRRTStar

def test_pendulum_planning():
    pendulum_system = Pendulum(input_limits=np.asarray([[-0.3],[0.3]]))
    goal_state = np.asarray([np.pi,0])
    def sampler():
        rnd = np.random.rand(2)
        rnd[0] = (rnd[0]-0.5)*3*np.pi
        rnd[1] = (rnd[1]-0.5)*1
        return rnd
    rrt = ContinuousSystem_RGRRTStar(pendulum_system, sampler, 1e-1)
    rrt.build_tree_to_goal_state(goal_state,stop_on_first_reach=True, allocated_time=np.inf)

if __name__=='__main__':
    test_pendulum_planning()