import pydrake
from rg_rrt_star.common.basic_rrt import *

class SymbolicSystem_Basic_RRT(BasicRRT):
    def __init__(self, sys, sampler, step_size, reached_goal_function):
        self.sys = sys
        self.step_size = step_size
        self.reached_goal_function=reached_goal_function
        def plan_collision_free_path(nearest_state, new_state):
            # sample the control space
            # TODO: support higher dimensions
            print(sys.input_limits[0,:])
            possible_inputs = np.linpace(*sys.input_limits[0,:], num=50)
            best_input = None
            for input in possible_inputs:

            # forward simulate with control space samples
            new_env = self.sys._state_to_env(nearest_state, u)
            self.sys.forward_step()

            # find the closest
        BasicRRT.__init__(self,self.sys.get_current_state(),sampler,reached_goal_function, plan_collision_free_path)