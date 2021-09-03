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
  <img src="https://user-images.githubusercontent.com/62132206/132067554-12fd77aa-967a-4d41-ada6-c2d88ebc2bce.gif?raw=true" width="500">
</p>

#### RRT + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/132067551-d9757bad-acf7-4005-884d-36709aaf1bc2.gif?raw=true" width="500">
</p>

#### Hybrid A* + Dubins Path

No reversing vs. Reversing

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/132067546-a508cbe8-0031-4cf2-afc3-758f1104ebe8.gif?raw=true" width="500">
  <img src="https://user-images.githubusercontent.com/62132206/132067538-f24867ff-74dc-42e8-90fe-2c45a2db7dca.gif?raw=true" width="500">
</p>
