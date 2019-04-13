# Reachability-Guided RRT*
Warning: support for `box_world` has been temporarily dropped and it will NOT run out of the box!

## Usage

### Installation
Clone this repository with `git clone --recursive` if you want to run `box_world` or `dubin_car` problems. The code depends on the repository [`bounding_box_closest_polytopes`](https://github.com/wualbert/bounding_box_closest_polytope.git "wualbert/bounding_box_closest_polytope") for axis-aligned bounding box representations.

### Running an example
All examples are located in the respective `/test` folders of each problem setup.

### Setting up a `dubin_car` problem
Please refer to the [this](https://github.mit.edu/rss2019-14/rg_rrt_star/blob/fcaa738f33fbe5be585b33ce36080c8b946d9add/dubin_car/dubin_car_rg_rrt_star.py#L298) part of the code on what information is required to setup the problem. In addition, `rg_rrt_star/dubin_car/tests/test_dubin_car.py` contains usage examples. Notice that `DC_Map` is NOT required--the planner works for other map representations.

### Solving for the path to a goal
Please refer to the function [`build_tree_to_goal_state`](https://github.mit.edu/rss2019-14/rg_rrt_star/blob/56f6e18789557be88fa36509cad7dccd0ff65220/common/rg_rrt_star.py#L223) and examples in `/test` folders for how to solve for the path after setting up the problem.

## Troubleshooting
If git submodules isn't working, `bounding_box_closest_polytopes` can be found [here](https://github.com/wualbert/bounding_box_closest_polytope.git "wualbert/bounding_box_closest_polytope")
