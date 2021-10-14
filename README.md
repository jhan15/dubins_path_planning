[![GitHub issues](https://img.shields.io/github/issues/jhan15/dubins_path_planning)](https://github.com/jhan15/dubins_path_planning/issues)
![GitHub last commit](https://img.shields.io/github/last-commit/jhan15/dubins_path_planning?color=ff69b4)

# dubins_path_planning

Implemented car path planning with RRT, Hybrid A*, and Dubins Path algorithms. The car model is

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
$ python3 hybrid_astar.py -heu 1 -r -e # A* heuristic + reverse + extra cost
```

## Results

#### Dubins Path

Pick the shortest obstacle-free dubins path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/133502943-b42a77ee-5152-43f3-9891-0c488773a05b.gif?raw=true" width="500">
</p>

#### RRT + Dubins Path

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/132485976-f1b545b8-4358-4e55-b73c-c65b9ff6c02d.gif?raw=true" width="500">
</p>

#### Hybrid A* + Dubins Path

w/o reversing

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/132485964-0c66ec2c-b8e8-4fdc-bf05-785b87a69320.gif?raw=true" width="500">
</p>

w/ reversing (blue tree)

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/132485946-9f0005d9-8aca-4a30-b0fa-94ef0fbdc06b.gif?raw=true" width="500">
</p>
