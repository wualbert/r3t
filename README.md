# R3T
Warning: support for `box_world` and `dubins_car` has been temporarily dropped and it will NOT run out of the box!

## Usage

#### Dependencies
The repository requires [`PyDrake with Gurobi`](https://drake.mit.edu/python_bindings.html), [`closest_polytope_algorithms`](https://github.com/wualbert/closest_polytope_algorithms.git), and [`polytope_symbolic_system`](https://github.com/wualbert/closest_polytope_algorithms.git) to run.

#### Installation
After installing [`PyDrake with Gurobi`](https://drake.mit.edu/python_bindings.html), clone [`closest_polytope_algorithms`](https://github.com/wualbert/closest_polytope_algorithms.git) and [`polytope_symbolic_system`](https://github.com/wualbert/closest_polytope_algorithms.git) to the same directory level as `r3t`.

#### Running an example
All examples are located in the respective `/test` folders of each problem setup.

#### Setting up a `dubin_car` problem
Please refer to the `DC_RGRRTStar.__init__` for what information is required to setup the problem. In addition, `rg_rrt_star/dubin_car/tests/test_dubin_car.py` contains usage examples. Notice that `DC_Map` is NOT required--the planner works for other map representations.

#### Solving for the path to a goal
Please refer to the function `build_tree_to_goal_state` and examples in `/test` folders for how to solve for the path after setting up the problem.
