import numpy as np
from math import sin, atan2, atan
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from car import SimpleCar
from environment import Environment
from test_cases.cases import TestCase
from dubins_path import DubinsPath
from lookup import Lookup
from utils.utils import distance, plot_a_car

from time import time


class Node:
    """ RRT tree node. """

    def __init__(self, pos, phi=0, steps=0):

        self.pos = pos
        self.phi = phi
        self.steps = steps
        self.parent = None
    
    def __eq__(self, other):

        return (self.pos == other.pos \
            and self.phi == other.phi \
            and self.steps == other.steps)
    
    def __hash__(self):

        return hash((self.pos, self.phi, self.steps))


class RRT:
    """ RRT + Dubins path algorithms. """

    def __init__(self, car, max_steps=50, pick_target=10, check_dubins=1):

        self.car = car
        self.max_steps = max_steps
        self.pick_target = pick_target
        self.check_dubins = check_dubins

        self.start = Node(self.car.start_pos)
        self.goal = Node(self.car.end_pos)

        self.dubins = DubinsPath(self.car)
        self.lookup = Lookup(self.car)
    
    def get_nearest_node(self, nodes, pick):
        """ Get the nearest node of a random pick. """

        dl = [distance(node.pos[:2], pick) for node in nodes]
        
        return nodes[dl.index(min(dl))]
    
    def get_steering_angle(self, pos, pick):
        """ Calculate steering angle. """

        v1 = np.array(pos[:2])
        v2 = np.array(pick)
        vb = v2 - v1
        
        b = np.linalg.norm(vb)
        lamda = atan2(vb[1], vb[0])
    
        phi = atan(2*self.car.l*sin(lamda-pos[2])/b)
        phi = max(min(phi, self.car.max_phi), -self.car.max_phi)
        
        return phi
    
    def backtracking(self, node):

        route = []
        while node.parent:
            route.append((node.pos, node.phi))
            node = node.parent
        
        return list(reversed(route))
    
    def search_path(self):
        """ Search path, return controls. """

        nodes = [self.start]
        final_node = None
        
        count = 0
        while True:
            count += 1

            if count % self.pick_target == 0:
                pick = self.goal.pos[:2]
            else:
                pick = self.car.random_pos(self.lookup)[:2]
            
            nearest = self.get_nearest_node(nodes, pick)

            if count % self.check_dubins == 0:
                solutions = self.dubins.find_tangents(nearest.pos, self.goal.pos)
                dubins_route, valid = self.dubins.best_tangent(solutions)
                
                if valid:
                    final_node = nearest
                    break

            phi = self.get_steering_angle(nearest.pos, pick)
            pos = nearest.pos
            
            for i in range(self.max_steps):
                pos = self.car.step(pos, phi)
            
            # check safety of route-----------------------
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(nearest.pos, pos)
            else:
                d, c, r = self.car.get_params(nearest.pos, phi)
                safe = self.dubins.is_turning_route_safe(nearest.pos, pos, d, c, r)
            # --------------------------------------------
            
            if not safe:
                continue
            
            new_node = Node(pos, phi, i+1)
            
            if new_node in nodes:
                continue
            
            new_node.parent = nearest
            nodes.append(new_node)
        
        route = self.backtracking(final_node) + dubins_route
        path = self.car.get_path(self.car.start_pos, route)
        print('Total iteration:', count)
        
        return path


def main():

    tc = TestCase()

    env = Environment(tc.obs3)

    car = SimpleCar(env, tc.start_pos, tc.end_pos)

    rrt = RRT(car)

    t = time()

    path = rrt.search_path()

    print('Total time: {}s'.format(round(time()-t, 3)))

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
