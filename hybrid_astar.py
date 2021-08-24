from math import pi, radians
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
import numpy as np

from grid import Grid
from car import SimpleCar
from environment import Environment
from dubins_path import DubinsPath
from test_cases.cases import TestCase
from utils.utils import distance, plot_a_car, mod_angle


class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_id, theta, pos):

        self.grid_id = grid_id
        self.theta = theta

        self.pos = pos
        self.g = None
        self.f = None
        self.parent = None

        self.phi = None
        self.steps = None
        self.dt = None


class SimpleState:
    """ Hash function for tree nodes. """

    def __init__(self, node):

        self.grid_id = node.grid_id
        self.theta = node.theta
    
    def __eq__(self, other):

        return (self.grid_id == other.grid_id \
            and self.theta == other.theta)
    
    def __hash__(self):

        return hash((self.grid_id, self.theta))
    

class HybridAstar:
    """ Hybrid A* search procedure. """

    def __init__(self, car, grid, theta_step=pi/12):
        
        self.car = car
        self.grid = grid
        self.theta_step = theta_step

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.dubins = DubinsPath(self.car)

        self.get_discretized_thetas()
    
    def get_discretized_thetas(self):
        """ Get all discretized headings in one grid. """

        self.thetas = [0]
        while True:
            theta = self.thetas[-1] + self.theta_step
            if theta > 2*pi:
                break
            
            self.thetas.append(theta)
    
    def construct_node(self, pos, root=False):
        """ Create node for a pos. """

        theta = pos[2]
        pos = pos[:2]
        if root:
            theta = mod_angle(self.start[2])
            theta = min(self.thetas, key=lambda x: abs(x-theta))
        
        grid_id = self.grid.to_grid_id(pos)

        node = Node(grid_id, theta, pos)

        return node
    
    def backtracking(self, node):

        controls = []
        while node.parent:
            controls.append((node.phi, node.steps, node.dt))
            node = node.parent
        
        return list(reversed(controls))
    
    def search_path(self):
        """ Hybrid A* pathfinding. """

        root = self.construct_node(self.start, root=True)
        root.g = float(0)
        root.f = self.heuristic_cost(root.pos)

        _closed = []
        _open = [root]

        states = [SimpleState(root)]

        while _open:
            best = min(_open, key=lambda x: x.f)

            _open.remove(best)
            _closed.append(best)

            # check dubins path
            pos = best.pos + [best.theta]
            solutions = self.dubins.find_tangents(pos, self.goal)
            dubins_controls = self.dubins.best_tangent(solutions)
            
            if dubins_controls:
                print('goal!')
                controls = self.backtracking(best)
                
                return controls + dubins_controls

            # construct neighbors

    
    def heuristic_cost(self, pos):
        """ Heuristic for remaining cost estimation. """
        
        return distance(pos, self.goal[:2])

    def get_path_length(self):
        pass


def main(grid_on=True):

    tc = TestCase()
    env = Environment(tc.obs3)
    car = SimpleCar(env, tc.start_pos, tc.end_pos)
    grid = Grid(env)
    ha = HybridAstar(car, grid)

    ha.search_path()

    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    controls = [
        (-radians(40), 150, 1e-2),
        (0, 200, 1e-2),
        (radians(15), 300, 1e-2)
    ]
    path = car.get_path(car.start_pos, controls)

    xl, yl = [], []
    carl = []
    for i in range(len(path)):
        xl.append(path[i]['pos'][0])
        yl.append(path[i]['pos'][1])
        carl.append(path[i]['model'][0])

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")
    if grid_on:
        ax.set_xticks(np.arange(0, env.lx, grid.grid_size))
        ax.set_yticks(np.arange(0, env.ly, grid.grid_size))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        plt.grid()
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.plot(start_state['pos'][0], start_state['pos'][1], 'ro', markersize=5)
    ax = plot_a_car(ax, end_state['model'])

    _path, = ax.plot([], [], color='lime', linewidth=1)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(path) + 1

    def animate(i):

        _path.set_data(xl[min(i, len(path)-1):], yl[min(i, len(path)-1):])

        sub_carl = carl[:min(i+1, len(path))]
        _carl.set_paths(sub_carl[::20])
        _carl.set_edgecolor('m')
        _carl.set_facecolor('None')
        _carl.set_alpha(0.2)

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']
        _car.set_paths(path[min(i, len(path)-1)]['model'])
        _car.set_edgecolor(edgecolor)
        _car.set_facecolor(facecolor)
        _car.set_zorder(3)

        return _path, _carl, _car

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=5,
                                  repeat=True, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
