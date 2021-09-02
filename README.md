
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

# pathfinding with Hybrid A* + Dubins Path (final shot)
$ python3 hybrid_astar.py --heu 0 # simple heuristic
                          --heu 1 # A* heuristic
```

## Results

#### Dubins Path

Pick the shortest obstacle-free dubins path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131924167-b09d1d31-0676-4fe2-8002-d2c251c1ef84.gif?raw=true">
</p>

#### RRT + Dubins Path

Pathfinding with RRT as branching and Dubins paths as final shot

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131924166-94441f7c-3f55-40d1-b5a9-a5c04d9b191a.gif?raw=true">
</p>

#### Hybrid A* + Dubins Path

Pathfinding with Hybrid A* as branching and Dubins paths as final shot

(no extra cost vs. extra cost)

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131924165-a2145aa0-35d0-4d24-9df2-0a1d85460d1b.gif?raw=true">
  <img src="https://user-images.githubusercontent.com/62132206/131924168-64db34a3-77a1-435d-aadf-ce7268268ec7.gif?raw=true">
</p>

Simple heuristic vs. A* heuristic

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/131811135-cf837e22-924b-4dd3-b47a-06ecf8b36236.png?raw=true" width="300">
  <img src="https://user-images.githubusercontent.com/62132206/131811131-74093f80-d5a8-4127-8fdf-2ec614a1cb5c.png?raw=true" width="300">
</p>
