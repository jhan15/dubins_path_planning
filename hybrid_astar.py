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
from lookup import Lookup
from test_cases.cases import TestCase
from utils.utils import distance, plot_a_car, mod_angle, get_discretized_thetas, round_theta

from time import time


class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):

        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.f = None
        self.parent = None
        self.phi = None


class State:
    """ Hash function for tree nodes. """

    def __init__(self, node):

        self.grid_pos = node.grid_pos
        self.pos = node.pos
        self.g = node.g
        self.f = node.f
        self.parent = node.parent
        self.phi = node.phi
    
    def __eq__(self, other):

        return (self.grid_pos == other.grid_pos)
    
    def __hash__(self):

        return hash((self.grid_pos))
    

class HybridAstar:
    """ Hybrid A* search procedure. """

    def __init__(self, car, grid, unit_theta=pi/12, drive_steps=40, dt=1e-2, check_dubins=1):
        
        self.car = car
        self.grid = grid
        self.unit_theta = unit_theta
        self.drive_steps = drive_steps
        self.dt = dt
        self.check_dubins = check_dubins

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)
        self.arc = self.drive_steps * self.dt
        self.phil = [-self.car.max_phi, 0, self.car.max_phi]

        self.dubins = DubinsPath(self.car)
        self.lookup = Lookup(self.car)

        self.thetas = get_discretized_thetas(self.unit_theta)
    
    def construct_node(self, pos, root=False):
        """ Create node for a pos. """

        theta = pos[2]
        pt = pos[:2]

        if root:
            theta = mod_angle(theta)
        
        theta = round_theta(theta, self.thetas)
        
        cell_id = self.grid.to_cell_id(pt)
        grid_pos = cell_id.tolist() + [theta]

        node = Node(grid_pos, pos)

        return node
        
    def heuristic_cost(self, pos):
        """ Heuristic for remaining cost estimation. """
        
        return distance(pos[:2], self.goal[:2])

    def get_children(self, node):
        """ Get successors from a state. """

        children = []
        for phi in self.phil:
            
            pos = node.pos
            for _ in range(self.drive_steps):
                pos = self.car.step(pos, phi)
                safe = self.car.is_pos_safe(pos, self.lookup)

                if not safe:
                    break
            
            if not safe:
                continue
            
            child = self.construct_node(pos)
            child.phi = phi
            child.parent = node
            child.g = node.g + self.arc
            child.f = child.g + self.heuristic_cost(child.pos)
            children.append(child)

        return children
    
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
        root.f = root.g + self.heuristic_cost(root.pos)

        closed_ = []
        open_ = [State(root)]

        count = 0
        while open_:
            count += 1
            best = min(open_, key=lambda x: x.f)

            open_.remove(best)
            closed_.append(best)

            # check dubins path
            self.check_dubins = max(1, int(2000/count))
            if count % self.check_dubins == 0:
                solutions = self.dubins.find_tangents(best.pos, self.goal)
                dubins_route, valid = self.dubins.best_tangent(solutions)
                
                if valid:
                    route = self.backtracking(best) + dubins_route
                    path = self.car.get_path(self.start, route)
                    
                    return path

            # construct neighbors
            children = self.get_children(best)

            for child in children:
                state = State(child)

                if state in closed_:
                    continue

                if state not in open_:
                    open_.append(state)
                elif state.g < open_[open_.index(state)].g:
                    open_.remove(state)
                    open_.append(state)

        return None


def main(grid_on=True):

    tc = TestCase()
    env = Environment(tc.obs3)
    car = SimpleCar(env, tc.start_pos, tc.end_pos)
    grid = Grid(env)
    hastar = HybridAstar(car, grid)

    t = time()
    path = hastar.search_path()

    print('Total time: {}s'.format(round(time()-t, 3)))

    if not path:
        print('No valid path!')
        return

    xl, yl = [], []
    carl = []
    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])

    end_state = car.get_car_state(car.end_pos)

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
    ax = plot_a_car(ax, end_state.model)

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
        _carl.set_color('m')
        _carl.set_alpha(0.1)

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']
        _car.set_paths(path[min(i, len(path)-1)].model)
        _car.set_edgecolor(edgecolor)
        _car.set_facecolor(facecolor)
        _car.set_zorder(3)

        return _path, _carl, _car

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=1,
                                  repeat=False, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
