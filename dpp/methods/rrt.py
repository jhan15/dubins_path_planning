import numpy as np
from math import sin, atan2, atan

from dpp.methods.dubins_path import DubinsPath
from dpp.utils.utils import distance


class Node:
    """ RRT tree node. """

    def __init__(self, pos, phi=0, steps=0):

        self.pos = pos
        self.phi = phi
        self.steps = steps
        self.parent = None
        self.branch = None
    
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
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.pos, node.phi, 1))
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
                pick = self.car.random_pos()[:2]
            
            nearest = self.get_nearest_node(nodes, pick)

            if count % self.check_dubins == 0:
                solutions = self.dubins.find_tangents(nearest.pos, self.goal.pos)
                dubins_route, cost, valid = self.dubins.best_tangent(solutions)
                
                if valid:
                    final_node = nearest
                    break

            phi = self.get_steering_angle(nearest.pos, pick)
            pos = nearest.pos
            branch = [pos[:2]]
            
            for i in range(self.max_steps):
                pos = self.car.step(pos, phi)
                branch.append(pos[:2])
            
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
            
            new_node.branch = branch
            new_node.parent = nearest
            nodes.append(new_node)
        
        route = self.backtracking(final_node) + dubins_route
        path = self.car.get_path(self.car.start_pos, route)
        print('Total iteration:', count)
        
        return path, nodes
