from math import cos, sin, tan, degrees, pi
from random import uniform
from matplotlib.patches import Rectangle, Arrow
from utils.utils import transform


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
    
    def random_pos(self):
        """ Generate a random pos. """
        
        while True:
            x = uniform(self.env.lx*0.1, self.env.lx*0.9)
            y = uniform(self.env.ly*0.1, self.env.ly*0.9)
            theta = uniform(-pi, pi)
            
            pos = [x, y, theta]
            state = self.get_car_state(pos)
            
            if self.env.obstacle_free(state['vertex']):
                break

        return pos
    
    def get_car_state(self, pos, phi=0):
        """ Get the car state according to the pos and steering angle. """

        x, y, theta = pos

        self.c1 = transform(x, y, 1.3*self.l, 0.4*self.l, theta, 1)
        self.c2 = transform(x, y, 1.3*self.l, 0.4*self.l, theta, 2)
        self.c3 = transform(x, y, 0.3*self.l, 0.4*self.l, theta, 3)
        self.c4 = transform(x, y, 0.3*self.l, 0.4*self.l, theta, 4)

        c_ = transform(x, y, self.l, 0.2*self.l, theta, 1)
        self.w1 = transform(c_[0], c_[1], 0.2*self.l, 0.1*self.l, theta+phi, 4)
        
        c_ = transform(x, y, self.l, 0.2*self.l, theta, 2)
        self.w2 = transform(c_[0], c_[1], 0.2*self.l, 0.1*self.l, theta+phi, 4)
        
        self.w3 = transform(x, y, 0.2*self.l, 0.1*self.l, theta, 3)
        self.w4 = transform(x, y, 0.2*self.l, 0.3*self.l, theta, 4)

        state = {
            'pos'   :   [x, y, theta],
            'vertex':   [self.c1, self.c2, self.c3, self.c4],
            'model' :   [
                Rectangle(self.c4, self.carl, self.carw, degrees(theta), fc='y', ec='k'),
                Rectangle(self.w1, self.whll, self.whlw, degrees(theta+phi), fc='k', ec='k'),
                Rectangle(self.w2, self.whll, self.whlw, degrees(theta+phi), fc='k', ec='k'),
                Rectangle(self.w3, self.whll, self.whlw, degrees(theta), fc='k', ec='k'),
                Rectangle(self.w4, self.whll, self.whlw, degrees(theta), fc='k', ec='k'),
                Arrow(x, y, 1.1*self.carl*cos(theta), 1.1*self.carl*sin(theta), width=0.1, color='r')
            ]
        }

        return state
    
    def step(self, pos, phi, dt=1e-2):
        """ Car dynamics. """

        x, y, theta = pos
        dx     = cos(theta)
        dy     = sin(theta)
        dtheta = tan(phi) / self.l

        x += dt*dx
        y += dt*dy
        theta += dt*dtheta

        return [x, y, theta]
    
    def get_path(self, pos, controls, rate=1e-2):
        """ Generate driving path according to control inputs. """
        
        path = []
        
        count = 0
        for phi, steps, dt in controls:
            freq = int(rate / dt)
            for _ in range(steps):
                if count % freq == 0:
                    car_state = self.get_car_state(pos, phi)
                    path.append(car_state)
                pos = self.step(pos, phi, dt)
                count += 1
        
        car_state = self.get_car_state(pos, phi)
        path.append(car_state)

        return path
