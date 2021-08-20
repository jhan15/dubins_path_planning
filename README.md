
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

# demonstrate dubins path
$ python3 dubins_path.py

# pathfinding with RRT + Dubins Path
$ python3 main.py
```

## Functions

#### Shortest obstacle-free Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/130232277-5a4aeed9-d459-45bf-9d42-44f83b29beae.gif?raw=true">
</p>

#### Pathfinding w/ RRT + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/130232281-ef374c2a-ce35-4eb3-998e-85b187b85e86.gif?raw=true">
</p>

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/130233967-58b18097-a9a7-4d06-8713-1d7aa615014e.gif?raw=true">
</p>
