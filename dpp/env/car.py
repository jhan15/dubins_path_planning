from math import cos, sin, tan, degrees, pi
from random import uniform
from matplotlib.patches import Rectangle, Arrow
from time import time

from dpp.utils.utils import transform, same_point


class State:

    def __init__(self, pos, model):

        self.pos = pos
        self.model = model


class SimpleCar:
    """ Car model and functions. """

    def __init__(self, env, start_pos=None, end_pos=None, l=0.5, max_phi=pi/5):

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
    
    def random_pos(self):
        """ Generate a random pos. """
        
        while True:
            x = uniform(self.env.lx*0.1, self.env.lx*0.9)
            y = uniform(self.env.ly*0.1, self.env.ly*0.9)
            theta = uniform(-pi, pi)
            
            pos = [x, y, theta]
            safe = self.is_pos_safe(pos)
            
            if safe:
                break

        return pos

    def get_params(self, pos, phi):
        """ Get parameters for turning route. """

        x, y, theta = pos

        r = self.l / abs(tan(phi))
        d = 1 if phi > 0 else -1
        id = 1 if phi > 0 else 2

        c = transform(x, y, 0, r, theta, id)

        return d, c, r
    
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
    
    def step(self, pos, phi, m=1, dt=1e-2):
        """ Car dynamics. """

        x, y, theta = pos
        dx     = cos(theta)
        dy     = sin(theta)
        dtheta = tan(phi) / self.l

        x     += m*dt*dx
        y     += m*dt*dy
        theta += m*dt*dtheta

        return [x, y, theta]
    
    def is_pos_safe(self, pos):
        """ Check pos safety. """

        vertex = self.get_car_bounding(pos)

        return self.env.rectangle_safe(vertex)
    
    def is_route_safe(self, pos, route):
        """ Check route safety. """

        safe = True

        for goal, phi, m in route:
            while True:
                pos = self.step(pos, phi, m)
                safe = self.is_pos_safe(pos)

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

        for goal, phi, m in route:
            while True:
                car_state = self.get_car_state(pos, phi)
                path.append(car_state)

                pos = self.step(pos, phi, m)

                if same_point(pos[:2], goal[:2]):
                    pos = goal
                    break
        
        car_state = self.get_car_state(pos, phi)
        path.append(car_state)

        return path
    
    def _get_path(self, pos, controls):
        """ Generate driving path according to control inputs. """
        
        path = []

        for phi, m, steps in controls:
            for _ in range(steps):
                car_state = self.get_car_state(pos, phi)
                path.append(car_state)
                pos = self.step(pos, phi, m)
        
        car_state = self.get_car_state(pos, phi)
        path.append(car_state)

        return path
