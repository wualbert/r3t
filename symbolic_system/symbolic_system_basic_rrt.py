import pydrake
from rg_rrt_star.common.basic_rrt import *

class SymbolicSystem_Basic_RRT(BasicRRT):
    def __init__(self, sys, sampler, step_size, reached_goal_function):
        self.sys = sys
        self.step_size = step_size
        self.reached_goal_function=reached_goal_function
        def plan_collision_free_path_towards(nearest_state, new_state):
            # sample the control space
            # TODO: support higher dimensions
            possible_inputs = np.linspace(*sys.input_limits[:,0], num=5)
            best_input = None
            best_new_state = None
            best_distance = np.inf
            for input in possible_inputs:
                new_potential_state = self.sys.forward_step(starting_state = np.atleast_1d(nearest_state), u=np.atleast_1d(input), modify_system=False, return_as_env=False, step_size=self.step_size)
                # print(nearest_state,new_potential_state,new_state)
                new_distance = np.linalg.norm(new_state - new_potential_state)
                # print(new_distance)
                if new_distance<best_distance:
                    best_input=input
                    best_new_state=new_potential_state
                    best_distance=new_distance
            return best_distance, best_new_state

        BasicRRT.__init__(self,self.sys.get_current_state(),sampler,reached_goal_function, plan_collision_free_path_towards)


class SymbolicSystem_RGRRT(RGRRT):
    def __init__(self, sys, sampler, step_size, reached_goal_function):
        self.sys = sys
        self.step_size = step_size
        self.reached_goal_function=reached_goal_function
        def plan_collision_free_path_towards(nearest_state, new_state):
            # sample the control space
            # TODO: support higher dimensions
            possible_inputs = np.linspace(*sys.input_limits[:,0], num=5)
            best_input = None
            best_new_state = None
            best_distance = np.inf
            for input in possible_inputs:
                new_potential_state = self.sys.forward_step(starting_state = np.atleast_1d(nearest_state), u=np.atleast_1d(input), modify_system=False, return_as_env=False, step_size=self.step_size)
                # print(nearest_state,new_potential_state,new_state)
                new_distance = np.linalg.norm(new_state - new_potential_state)
                # print(new_distance)
                if new_distance<best_distance:
                    best_input=input
                    best_new_state=new_potential_state
                    best_distance=new_distance
            return best_distance, best_new_state

        RGRRT.__init__(self,self.sys.get_current_state(),sampler,reached_goal_function, plan_collision_free_path_towards)

    def get_reachable_states(self, state):
        possible_inputs = np.linspace(*self.sys.input_limits[:, 0], num=5)
        possible_new_states = []
        for input in possible_inputs:
            possible_new_states.append(self.sys.forward_step(starting_state=np.atleast_1d(state),
                                                        u=np.atleast_1d(input), modify_system=False,
                                                        return_as_env=False, step_size=self.step_size))
        return possible_new_states