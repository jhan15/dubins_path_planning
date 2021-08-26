from math import cos, sin, tan, degrees, pi
from random import uniform
from matplotlib.patches import Rectangle, Arrow
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.animation as animation

from environment import Environment
from lookup import Lookup
from test_cases.cases import TestCase
from utils.utils import transform, same_point

from time import time


class State:

    def __init__(self, pos, model):

        self.pos = pos
        self.model = model


class SimpleCar:
    """ Car model and functions. """

    def __init__(self, env, start_pos=None, end_pos=None, l=0.5, max_phi=pi/4):

        self.env = env
        self.l = float(l)
        self.max_phi = max_phi

        self.carl = 1.6 * self.l
        self.carw = 0.8 * self.l

        self.whll = 0.4 * self.l
        self.whlw = 0.2 * self.l

        if start_pos:
            self.start_pos = start_pos
        else:
            self.start_pos = self.random_pos()
        
        if end_pos:
            self.end_pos = end_pos
        else:
            self.end_pos = self.random_pos()
    
    def random_pos(self, lookup):
        """ Generate a random pos. """
        
        while True:
            x = uniform(self.env.lx*0.1, self.env.lx*0.9)
            y = uniform(self.env.ly*0.1, self.env.ly*0.9)
            theta = uniform(-pi, pi)
            
            pos = [x, y, theta]
            safe = self.is_pos_safe(pos, lookup)
            
            if safe:
                break

        return pos
    
    def get_car_bounding(self, pos):
        """ Get the bounding rectangle of car. """

        x, y, theta = pos

        self.c1 = transform(x, y, 1.3*self.l, 0.4*self.l, theta, 1)
        self.c2 = transform(x, y, 1.3*self.l, 0.4*self.l, theta, 2)
        self.c3 = transform(x, y, 0.3*self.l, 0.4*self.l, theta, 3)
        self.c4 = transform(x, y, 0.3*self.l, 0.4*self.l, theta, 4)

        vertex = [self.c1.tolist(), self.c2.tolist(), self.c4.tolist(), self.c3.tolist()]

        return vertex
    
    def get_car_state(self, pos, phi=0):
        """ Get the car state according to the pos and steering angle. """
        
        x, y, theta = pos

        pos = [x, y, theta]
        self.get_car_bounding(pos)

        c_      = transform(x, y, self.l, 0.2*self.l, theta, 1)
        self.w1 = transform(c_[0], c_[1], 0.2*self.l, 0.1*self.l, theta+phi, 4)
        
        c_      = transform(x, y, self.l, 0.2*self.l, theta, 2)
        self.w2 = transform(c_[0], c_[1], 0.2*self.l, 0.1*self.l, theta+phi, 4)
        
        self.w3 = transform(x, y, 0.2*self.l, 0.1*self.l, theta, 3)
        self.w4 = transform(x, y, 0.2*self.l, 0.3*self.l, theta, 4)
        
        model = [
            Rectangle(self.c4, self.carl, self.carw, degrees(theta), fc='y', ec='k'),
            Rectangle(self.w1, self.whll, self.whlw, degrees(theta+phi), fc='k', ec='k'),
            Rectangle(self.w2, self.whll, self.whlw, degrees(theta+phi), fc='k', ec='k'),
            Rectangle(self.w3, self.whll, self.whlw, degrees(theta), fc='k', ec='k'),
            Rectangle(self.w4, self.whll, self.whlw, degrees(theta), fc='k', ec='k'),
            Arrow(x, y, 1.1*self.carl*cos(theta), 1.1*self.carl*sin(theta), width=0.1, color='r')
        ]

        state = State(pos, model)

        return state
    
    def step(self, pos, phi, dt=1e-2):
        """ Car dynamics. """

        x, y, theta = pos
        dx     = cos(theta)
        dy     = sin(theta)
        dtheta = tan(phi) / self.l

        x     += dt*dx
        y     += dt*dy
        theta += dt*dtheta

        return [x, y, theta]
    
    def is_pos_safe(self, pos, lookup):
        """ Check pos safety by lookup table. """

        lookup_vertex = lookup.transform_state(pos)

        return self.env.safe(lookup_vertex)
    
    def is_route_safe(self, pos, route, lookup):
        """ Check route safety by lookup table. """

        safe = True

        for goal, phi in route:
            while True:
                pos = self.step(pos, phi)
                safe = self.is_pos_safe(pos, lookup)

                if not safe:
                    break

                if same_point(pos[:2], goal[:2]):
                    pos = goal
                    break
            
            if not safe:
                break

        return safe

    
    def get_path(self, pos, route):
        """ Generate path according to route. """

        path = []

        for goal, phi in route:
            while True:
                car_state = self.get_car_state(pos, phi)
                path.append(car_state)

                pos = self.step(pos, phi)

                if same_point(pos[:2], goal[:2]):
                    pos = goal
                    break
        
        car_state = self.get_car_state(pos, phi)
        path.append(car_state)

        return path
    
    def _get_path(self, pos, controls):
        """ Generate driving path according to control inputs. """
        
        path = []

        for phi, steps in controls:
            for _ in range(steps):
                car_state = self.get_car_state(pos, phi)
                path.append(car_state)
                pos = self.step(pos, phi)
        
        car_state = self.get_car_state(pos, phi)
        path.append(car_state)

        return path


def main():

    # test cases
    tc = TestCase()

    # map w/ obstacles
    env = Environment()

    # car w/ initial and target poses
    car = SimpleCar(env, tc.start_pos, tc.end_pos)

    # example controls to demonstrate car dynamics
    controls = [
        (-pi/8, 150),
        (0, 200),
        (-pi/8, 300),
        (0, 300),
        (pi/4, 100),
        (-pi/4, 100),
        (pi/4, 100)
    ]

    path = car._get_path(car.start_pos, controls)

    xl, yl = [], []
    carl = []
    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])

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

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=5,
                                  repeat=False, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
