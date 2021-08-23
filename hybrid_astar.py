from math import pi
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from grid import Grid
from car import SimpleCar
from environment import Environment
from dubins_path import DubinsPath
from test_cases.cases import TestCase
from utils.utils import distance, plot_a_car, mod_angle


class Node:
    """ Hybrid A* tree node. """

    def __init__(self, pos_grid, theta, pos):

        self.pos_grid = pos_grid
        self.theta = theta

        self.pos = pos
        self.g = None
        self.f = None
        self.parent = None


class SimpleState:
    """ Hash function for tree nodes. """

    def __init__(self, node):

        self.pos_grid = node.pos_grid
        self.theta = node.theta
    
    def __eq__(self, other):

        return (self.pos_grid == other.pos_grid \
            and self.theta == other.theta)
    
    def __hash__(self):

        return hash((self.pos_grid, self.theta))
    

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
        
        pos_grid = self.grid.grid_id(pos)

        node = Node(pos_grid, theta, pos)

        return node
    
    def search_path(self):
        """ Hybrid A* pathfinding. """
        
        controls = []

        root = self.construct_node(self.start, root=True)
        root.g = 0
        root.f = self.heuristic_cost(root.pos)

        _closed = []
        _open = [root]

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
                # return controls + dubins_controls
                break

            # construct neighbors

    
    def heuristic_cost(self, pos):
        """ Heuristic for remaining cost estimation. """
        
        return distance(pos, self.goal[:2])

    def get_path_length(self):
        pass


def main():

    tc = TestCase()
    env = Environment(tc.obs1)
    car = SimpleCar(env, tc.start_pos, tc.end_pos)
    grid = Grid(env)
    ha = HybridAstar(car, grid)

    ha.search_path()

    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax = plot_a_car(ax, start_state['model'])
    ax = plot_a_car(ax, end_state['model'])

    # plt.show()


if __name__ == '__main__':
    main()
