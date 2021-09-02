
[![GitHub issues](https://img.shields.io/github/issues/jhan15/dubins_path_planning)](https://github.com/jhan15/dubins_path_planning/issues)
![GitHub last commit](https://img.shields.io/github/last-commit/jhan15/dubins_path_planning?color=ff69b4)

# dubins_path_planning

In this project, we implemented car path planning methods with RRT, Hybrid A*, and Dubins Path algorithms. The car model is

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
 - `phi ∈ [-pi/5, pi/5]`: steering angle (w.r.t. the direction of travel).

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

# pathfinding with hybrid a* + Dubins Path (final shot)
$ python3 hybrid_astar.py
```

## Results

#### Shortest obstacle-free Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131809298-f4fde536-23a1-42b1-82ee-ddaf269708ae.gif?raw=true" width="200">
</p>

#### Pathfinding w/ RRT + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131809295-34b14b96-6d79-4872-a853-11b02fa94592.gif?raw=true">
</p>

#### Pathfinding w/ Hybrid A* + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131809755-9a8bc23c-2ba0-44a5-b417-5fdd12bd38e5.gif?raw=true">
</p>
