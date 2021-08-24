from math import pi, tan, sin
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
from utils.utils import distance, plot_a_car, mod_angle, arc_length


class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):

        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.f = None
        self.parent = None
        self.phi = None


class SimpleState:
    """ Hash function for tree nodes. """

    def __init__(self, node):

        self.grid_pos = node.grid_pos
    
    def __eq__(self, other):

        return (self.grid_pos == other.grid_pos)
    
    def __hash__(self):

        return hash((self.grid_pos))
    

class HybridAstar:
    """ Hybrid A* search procedure. """

    def __init__(self, car, grid, unit_theta=pi/12, drive_steps=40):
        
        self.car = car
        self.grid = grid
        self.unit_theta = unit_theta
        self.drive_steps = drive_steps

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)
        self.dubins = DubinsPath(self.car)

        self.get_discretized_thetas()
    
    def get_discretized_thetas(self):
        """ Get all discretized headings in one grid. """

        self.thetas = [0]

        while True:
            theta = self.thetas[-1] + self.unit_theta
            if theta > 2*pi:
                break
            
            self.thetas.append(theta)
    
    def round_theta(self, theta):

        return min(self.thetas, key=lambda x: abs(x-theta))
    
    def construct_node(self, pos, root=False):
        """ Create node for a pos. """

        theta = pos[2]
        pt = pos[:2]

        if root:
            theta = self.round_theta(mod_angle(theta))
        
        cell_id = self.grid.to_cell_id(pt)
        grid_pos = cell_id.tolist() + [theta]

        node = Node(grid_pos, pos)

        return node
        
    def heuristic_cost(self, pos):
        """ Heuristic for remaining cost estimation. """
        
        return distance(pos[:2], self.goal[:2])

    def motion_primitive(self, node):
        pass
    
    def backtracking(self, node):

        route = []
        while node.parent:
            route.append((node.pos, node.phi))
            node = node.parent
        
        return list(reversed(route))
    
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
            solutions = self.dubins.find_tangents(best.pos, self.goal)
            dubins_path, valid = self.dubins.best_tangent(solutions)
            
            if valid:
                print('goal!')
                route = self.backtracking(best)
                # path, _ = self.car.get_path(self.start, route)
                
                return dubins_path

            # construct neighbors


def main(grid_on=True):

    tc = TestCase()
    env = Environment(tc.obs3)
    car = SimpleCar(env, tc.start_pos, tc.end_pos)
    grid = Grid(env)
    ha = HybridAstar(car, grid)

    path = ha.search_path()

    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    controls = [
        (car.max_phi, 40),
        (0, 40),
        (-car.max_phi, 40)
    ]

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")

    if grid_on:
        ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
        ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        plt.grid()
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=5)
    ax = plot_a_car(ax, start_state['model'])

    for c in controls:
        path = car._get_path(car.start_pos, [c])

        xl, yl = [], []
        carl = []
        for i in range(len(path)):
            xl.append(path[i]['pos'][0])
            yl.append(path[i]['pos'][1])
            carl.append(path[i]['model'][0])
        
        ax.plot(xl, yl, 'b-', linewidth=1)

    plt.show()


if __name__ == '__main__':
    main()
