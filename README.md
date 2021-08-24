
[![GitHub issues](https://img.shields.io/github/issues/jhan15/dubins_path_planning)](https://github.com/jhan15/dubins_path_planning/issues)
![GitHub last commit](https://img.shields.io/github/last-commit/jhan15/dubins_path_planning?color=ff69b4)

# dubins_path_planning

In this project, we implemented a car path planning method with [Rapidly exploring Random Tree (RRT)](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) and [Dubins Path](https://en.wikipedia.org/wiki/Dubins_path) algorithms. The car model is

<p align="center">
  <img src="https://github.com/jhan15/dubins_path_planning/blob/master/images/car_model.png?raw=true" width="300">
</p>

with the dynamics

```python
x[t+1]     = x[t]     + cos(theta[t])
y[t+1]     = y[t]     + sin(theta[t])
theta[t+1] = theta[t] + tan(phi[t]) / L
```

The state variables are:
 - `x`: horizontal position
 - `y`: vertical position
 - `theta`: heading angle (direction of travel)

The control variable is:
 - `phi ∈ [-pi/4, pi/4]`: steering angle (w.r.t. the direction of travel).

## Usage

```bash
$ git clone https://github.com/jhan15/dubins_path_planning.git
$ cd dubins_path_planning

# demonstarte car dynamics
$ python3 car.py

# demonstrate dubins path (shortest obstacle-free)
$ python3 dubins_path.py

# pathfinding with RRT + Dubins Path (final shot)
$ python3 rrt.py
```

## Functions

#### Shortest obstacle-free Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/130496974-a36ccf19-6248-405c-a7fe-b19d86ef798f.gif?raw=true" width="500">
</p>

#### Pathfinding w/ RRT + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/130496983-a1f88c80-8993-43db-9533-0cc72c060139.gif?raw=true" width="500">
</p>
