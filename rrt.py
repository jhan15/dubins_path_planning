import numpy as np
from math import sin, cos
from dubins_path import DubinsPath


class Node:
    """ RRT tree node. """

    def __init__(self, pos, phi=0, steps=0, dt=1e-2):

        self.pos = pos
        self.phi = phi
        self.steps = steps
        self.dt = dt
        self.parent = None


class SimpleState:
    """ Hash function for tree nodes. """

    def __init__(self, node):

        self.pos = node.pos
        self.phi = node.phi
        self.steps = node.steps
        self.dt = node.dt
    
    def __eq__(self, other):

        return (self.pos == other.pos \
            and self.phi == other.phi \
            and self.steps == other.steps \
            and self.dt == other.dt)
    
    def __hash__(self):

        return hash((self.pos, self.phi, self.steps, self.dt))


class RRT:
    """ RRT + Dubins path algorithms. """

    def __init__(self, car, max_steps=50):

        self.car = car
        self.max_steps = max_steps
        self.pick_target_freq = 10

        self.start = Node(self.car.start_pos)
        self.goal = Node(self.car.end_pos)

        self.dubins = DubinsPath(self.car)
    
    def get_nearest_node(self, nodes, pick):
        """ Get the nearest node of a random pick. """

        d_2 = [(node.pos[0]-pick[0])**2 + (node.pos[1]-pick[1])**2 for node in nodes]
        
        return nodes[d_2.index(min(d_2))]
    
    def get_steering_angle(self, pos, pick):
        """ Calculate steering angle. """

        v = [cos(pos[2]), sin(pos[2])]
        r = [pick[0] - pos[0], pick[1] - pos[1]]

        cross = np.cross(v, r)
        dot = np.dot(v, r)
        phi = np.arctan2(cross, dot)

        phi = max(min(phi, self.car.max_phi), -self.car.max_phi)
        
        return phi
    
    def search_path(self):
        """ Search path, return controls. """

        # check dubins path for start and target
        solutions = self.dubins.find_tangents(self.start.pos, self.goal.pos)
        dubins_controls = self.dubins.best_tangent(solutions)
        
        if dubins_controls:
            return dubins_controls

        # rrt
        controls = []

        nodes = [self.start]
        states = [SimpleState(self.start)]
        final_node = None
        
        count = 0
        while True:
            count += 1
            if count % self.pick_target_freq == 0:
                pick = [self.goal.pos[0], self.goal.pos[1]]
            else:
                pick = self.car.random_pos()[:2]
            
            nearest = self.get_nearest_node(nodes, pick)
            phi = self.get_steering_angle(nearest.pos, pick)
            pos = nearest.pos
            
            for i in range(self.max_steps):
                pos = self.car.step(pos, phi)
                car_state = self.car.get_car_state(pos, phi)
                
                safe = self.car.env.safe(car_state['vertex'])

                if not safe:
                    break
            
            if not safe:
                continue
            
            new_node = Node(pos, phi, i+1)
            new_state = SimpleState(new_node)
            
            if new_state in states:
                continue
            
            states.append(new_state)
            new_node.parent = nearest
            nodes.append(new_node)

            # check dubins path of new node
            solutions = self.dubins.find_tangents(pos, self.goal.pos)
            dubins_controls = self.dubins.best_tangent(solutions)
            
            if dubins_controls:
                final_node = new_node
                break
        
        current = final_node
        while current.parent:
            controls.append((current.phi, current.steps, current.dt))
            current = current.parent
        
        controls = list(reversed(controls)) + dubins_controls
        
        return controls
