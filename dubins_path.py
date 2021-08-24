from math import tan, atan2, acos, pi
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from car import SimpleCar
from environment import Environment
from test_cases.cases import TestCase
from utils.utils import transform, directional_theta, plot_a_car


class Params:
    """ Store parameters for different dubins paths. """

    def __init__(self, d):

        self.d = d
        self.t1 = None
        self.t2 = None
        self.len = None


class DubinsPath:
    """
    Consider four dubins paths
    - LSL
    - LSR
    - RSL
    - RSR
    and find the shortest obstacle-free one.
    To achieve a reasonable accuracy, set dt=1e-4.
    """

    def __init__(self, car):

        self.car = car
        self.r = self.car.l / tan(self.car.max_phi)
        
        # turn left: 1, turn right: -1
        self.direction = {
            'RSL': [-1, 1],
            'LSL': [1, 1],
            'LSR': [1, -1],
            'RSR': [-1, -1]
        }
    
    def find_tangents(self, start_pos, end_pos):
        """ Find the tangents of four dubins paths. """

        self.start_pos = start_pos
        self.end_pos = end_pos

        x1, y1, theta1 = start_pos
        x2, y2, theta2 = end_pos
        
        self.s = np.array(start_pos[:2])
        self.e = np.array(end_pos[:2])
        
        self.lc1 = transform(x1, y1, 0, self.r, theta1, 1)
        self.rc1 = transform(x1, y1, 0, self.r, theta1, 2)
        self.lc2 = transform(x2, y2, 0, self.r, theta2, 1)
        self.rc2 = transform(x2, y2, 0, self.r, theta2, 2)
        
        solutions = [self._LSL(), self._LSR(), self._RSL(), self._RSR()]
        solutions = [s for s in solutions if s is not None]
        solutions.sort(key=lambda x: x.len, reverse=False)
        
        return solutions
    
    def get_params(self, dub, c1, c2, t1, t2):
        """ Calculate the dubins path length. """
        
        v1 = self.s - c1
        v2 = t1     - c1
        v3 = t2     - t1
        v4 = t2     - c2
        v5 = self.e - c2

        delta_theta1 = directional_theta(v1, v2, dub.d[0])
        delta_theta2 = directional_theta(v4, v5, dub.d[1])

        arc1    = abs(delta_theta1*self.r)
        tangent = np.linalg.norm(v3)
        arc2    = abs(delta_theta2*self.r)

        theta = self.start_pos[2] + delta_theta1

        dub.t1 = t1.tolist() + [theta]
        dub.t2 = t2.tolist() + [theta]
        dub.len = arc1 + tangent + arc2
        
        return dub
    
    def _LSL(self):

        lsl = Params(self.direction['LSL'])

        cline = self.lc2 - self.lc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) - acos(0)

        t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta, 1)

        lsl = self.get_params(lsl, self.lc1, self.lc2, t1, t2)

        return lsl

    def _LSR(self):

        lsr = Params(self.direction['LSR'])

        cline = self.rc2 - self.lc1
        R = np.linalg.norm(cline) / 2

        if R < 2*self.r:
            return None
        
        theta = atan2(cline[1], cline[0]) - acos(self.r/R)

        t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta+pi, 1)

        lsr = self.get_params(lsr, self.lc1, self.rc2, t1, t2)

        return lsr

    def _RSL(self):

        rsl = Params(self.direction['RSL'])

        cline = self.lc2 - self.rc1
        R = np.linalg.norm(cline) / 2

        if R < 2*self.r:
            return None
        
        theta = atan2(cline[1], cline[0]) + acos(self.r/R)

        t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta+pi, 1)

        rsl = self.get_params(rsl, self.rc1, self.lc2, t1, t2)

        return rsl

    def _RSR(self):

        rsr = Params(self.direction['RSR'])

        cline = self.rc2 - self.rc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) + acos(0)

        t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta, 1)

        rsr = self.get_params(rsr, self.rc1, self.rc2, t1, t2)

        return rsr
    
    def best_tangent(self, solutions):
        """ Get the shortest obstacle-free dubins path. """

        for s in solutions:
            route = self.get_route(s)
            path, safe = self.car.get_path(self.start_pos, route, 1e-4, True)

            if safe:
                break
        
        return path, safe
    
    def get_route(self, s):
        """ Get the route of dubins path. """

        phi1 = self.car.max_phi if s.d[0] == 1 else -self.car.max_phi
        phi2 = self.car.max_phi if s.d[1] == 1 else -self.car.max_phi

        phil = [phi1, 0, phi2]
        goal = [s.t1, s.t2, self.end_pos]
        
        return list(zip(goal, phil))


def main():

    # test cases
    tc = TestCase()

    # map w/ obstacles
    env = Environment(tc.obs1)

    # car w/ initial and target poses
    car = SimpleCar(env, tc.start_pos, tc.end_pos)

    # dubins path
    dubins = DubinsPath(car)

    # shortest obstacle-free dubins path
    solutions = dubins.find_tangents(car.start_pos, car.end_pos)
    path, safe = dubins.best_tangent(solutions)
    
    if not safe:
        print('No valid dubins path!')
        return

    carl = []
    for i in range(len(path)):
        carl.append(path[i].model[0])

    end_state = car.get_car_state(car.end_pos)

    tangents = []
    for s in solutions:
        tangents.append([s.t1[:2], s.t2[:2]])
    lc = LineCollection(tangents, color='k', linewidth=1)
    
    lcircle1 = plt.Circle(dubins.lc1, dubins.r, fc='None', ec='k')
    rcircle1 = plt.Circle(dubins.rc1, dubins.r, fc='None', ec='k')
    lcircle2 = plt.Circle(dubins.lc2, dubins.r, fc='None', ec='k')
    rcircle2 = plt.Circle(dubins.rc2, dubins.r, fc='None', ec='k')

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.add_collection(lc)
    ax.add_patch(lcircle1)
    ax.add_patch(rcircle1)
    ax.add_patch(lcircle2)
    ax.add_patch(rcircle2)
    
    ax = plot_a_car(ax, end_state.model)

    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _car = PatchCollection([])
    ax.add_collection(_car)
    frames = len(path) + 1

    def animate(i):

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

        return _carl, _car

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=5,
                                  repeat=False, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
