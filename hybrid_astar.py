from math import pi

from grid import Grid
from car import SimpleCar
from environment import Environment
from dubins_path import DubinsPath
from test_cases.cases import TestCase


class Node:
    """ Hybrid A* tree node. """

    def __init__(self, x_grid, theta, x):

        self.x_grid = x_grid
        self.theta = theta
        self.x = x
        self.g = None
        self.f = None
        self.parent = None


class SimpleState:
    """ Hash function for tree nodes. """

    def __init__(self, node):

        self.x_grid = node.x_grid
        self.theta = node.theta
    
    def __eq__(self, other):

        return (self.x_grid == other.x_grid \
            and self.theta == other.theta)
    
    def __hash__(self):

        return hash((self.x_grid, self.theta))
    

class HybridAstar:
    """ Hybrid A* search procedure. """

    def __init__(self, car, grid, theta_step=pi/12):
        
        self.car = car
        self.grid = grid
        self.theta_step = theta_step
        self.get_discretized_thetas()
    
    def get_discretized_thetas(self):

        self.thetas = [0]
        while True:
            theta = self.thetas[-1] + self.theta_step
            if theta > 2*pi:
                break
            
            self.thetas.append(theta)


def main():

    ha = HybridAstar()
    print(ha.thetas)


if __name__ == '__main__':
    main()
