from math import pi

from grid import Grid
from car import SimpleCar
from environment import Environment
from dubins_path import DubinsPath
from test_cases.cases import TestCase
from utils.utils import distance


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

        self.get_discretized_thetas()
    
    def get_discretized_thetas(self):
        """ Get all discretized headings in one grid. """

        self.thetas = [0]
        while True:
            theta = self.thetas[-1] + self.theta_step
            if theta > 2*pi:
                break
            
            self.thetas.append(theta)
    
    def search_path(self):
        """ Hybrid A* pathfinding. """

    
    def heuristic_cost(self, pos):
        """ Heuristic for remaining cost estimation. """
        
        return distance(pos, self.goal[:2])

    def get_path_length(self):
        pass


def main():

    ha = HybridAstar()
    print(ha.thetas)


if __name__ == '__main__':
    main()
