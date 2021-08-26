from math import pi

from utils.utils import get_discretized_thetas, round_theta, mod_angle


class Params:

    def __init__(self, theta, vertex):

        self.theta = theta
        self.vertex = vertex


class Lookup:

    def __init__(self, car, unit_theta=pi/36):

        self.car = car
        self.unit_theta = unit_theta

        self.thetas = get_discretized_thetas(self.unit_theta)
        self.create_lookup()

    def create_lookup(self):

        self.lookup = []

        for theta in self.thetas:
            pos = [0, 0, theta]
            vertex = self.car.get_car_bounding(pos)
            param = Params(theta, vertex)
            
            self.lookup.append(param)
    
    def transform_state(self, pos):

        theta = mod_angle(pos[2])
        theta = round_theta(theta, self.thetas)
        param = next((p for p in self.lookup if p.theta == theta), None)

        vertex = []

        for i in range(len(param.vertex)):
            vertex.append([param.vertex[i][0]+pos[0], param.vertex[i][1]+pos[1]])

        return vertex
