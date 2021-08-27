
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

# pathfinding with hybrid a* + Dubins Path (final shot)
$ python3 hybrid_astar.py
```

## Results

#### Shortest obstacle-free Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131178331-59925cc4-12ec-48c8-bb06-365731384238.gif?raw=true" width="500">
</p>

#### Pathfinding w/ RRT + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131178310-022056a3-c6f1-46f7-abb9-61019e9e4e72.gif?raw=true" width="500">
</p>

#### Pathfinding w/ Hybrid A* + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131178320-4c993399-3121-4092-8061-09336b5e36f8.gif?raw=true" width="500">
</p>
