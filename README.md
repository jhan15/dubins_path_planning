
[![GitHub issues](https://img.shields.io/github/issues/jhan15/dubins_path_planning)](https://github.com/jhan15/dubins_path_planning/issues)
![GitHub last commit](https://img.shields.io/github/last-commit/jhan15/dubins_path_planning?color=ff69b4)

# dubins_path_planning

## Description

In this project, we implement a robotic path planning method with [Rapidly exploring Random Tree (RRT)](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) algorithm in order to drive a Dubins car, so the path found by the algorithm is drivable by design. We simplify the kinematics for wheeled systems, considering the car as a point

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/129100045-7c7781eb-74a8-4ed0-add7-009af42e6ace.png?raw=true" width="300">
</p>

with the dynamics

```python
x[t+1]     = x[t]     + cos(theta[t])
y[t+1]     = y[t]     + sin(theta[t])
theta[t+1] = theta[t] + tan(phi[t])
```

from an initial position `(x0,y0)` to a target position `(xt, yt)`, while avoiding both collisions with obstacles and venturing out of bounds.

The state variables are:
 - `x`: horizontal position
 - `y`: vertical position
 - `theta`: heading angle (direction of travel)

And, the sole control variable is the steering angle `phi ∈ [-pi/4, pi/4]` (with respect to the direction of travel).

We consider two types of obstacles:
 - **Circular** — reach the target position with circular obstacles;
 - **Line** — reach the target position with line obstacles.

**Note**:
- we refer to the *state* as `(x, y, theta)` and the *position* as `(x, y)`;
- the line obstacles are represented by a series of circular obstacles;
- the initial and target positions are randomised;
- the integration will stop if the path goes into an obstacle, out of bounds, or within 0.2 meters of the target;
- each steering angle `controls[i]` is considered to be constant between `times[i]` and `times[i+1]`, so `controls` must be one element shorter than `times`, i.e. `len(controls) == len(times) - 1`;
- the initial time must be zero, i.e. `times[0] == 0`;
- the time list must be spaced by `≥0.01` seconds;
- each steering angle must be admissible, i.e. `-pi/4 <= controls[i] <= pi/4`;
- the time sequence must increase, i.e. `times[i+1] > times[i]`;
- the intial heading angle in evaluation is zero, i.e. `theta=0`.

## Usage

```bash
$ git clone https://github.com/jhan15/path_planning_dubins.git
$ cd path_planning_dubins
$ python3 main.py -case 12 -steps 100 -a # case 12, control steps 100, with animation
                                         # 0 <= case <= 12
```

## Results

#### Path

<p align="center">
  <img src="https://github.com/jhan15/path_planning_dubins/blob/master/images/path.png?raw=true" width="600">
</p>

#### Branching

<p align="center">
  <img src="https://github.com/jhan15/path_planning_dubins/blob/master/images/branching.gif?raw=true">
</p>

#### Complete RRT

<p align="center">
  <img src="https://github.com/jhan15/path_planning_dubins/blob/master/images/rrt.gif?raw=true">
</p>
