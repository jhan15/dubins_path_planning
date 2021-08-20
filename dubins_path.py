from math import tan, atan2, acos, pi
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from car import SimpleCar
from environment import Environment
from utils.utils import transform, calculate_arc_len, same_point, plot_a_car


class Params:

    def __init__(self, d):

        self.d = d
        self.t1 = None
        self.t2 = None
        self.path_len = None


class DubinsPath:

    def __init__(self, car):

        self.car = car
        self.r = self.car.l / tan(self.car.max_phi)
        
        self.direction = {
            'RSL': [-1, 1],
            'LSL': [1, 1],
            'LSR': [1, -1],
            'RSR': [-1, -1]
        }
    
    def find_tangents(self, start_pos, end_pos):

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
        
        solutions = [self._RSL(), self._LSL(), self._LSR(), self._RSR()]
        solutions.sort(key=lambda x: x.path_len, reverse=False)
        
        return solutions
    
    def best_tangent(self, solutions):

        for s in solutions:
            controls, safe = self.get_dubins_controls(s)
            if safe:
                break
        
        if not safe:
            return None
        
        return controls
    
    def _RSL(self):

        rsl = Params(self.direction['RSL'])

        cline = self.lc2 - self.rc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) + acos(self.r/R)

        rsl.t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        rsl.t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta+pi, 1)

        rsl.path_len = self.path_length(rsl.d, self.rc1, self.lc2, rsl.t1, rsl.t2)

        return rsl
    
    def _LSL(self):

        lsl = Params(self.direction['LSL'])

        cline = self.lc2 - self.lc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) - acos(0)

        lsl.t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        lsl.t2 = transform(self.lc2[0], self.lc2[1], self.r, 0, theta, 1)

        lsl.path_len = self.path_length(lsl.d, self.lc1, self.lc2, lsl.t1, lsl.t2)

        return lsl
    
    def _LSR(self):

        lsr = Params(self.direction['LSR'])

        cline = self.rc2 - self.lc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) - acos(self.r/R)

        lsr.t1 = transform(self.lc1[0], self.lc1[1], self.r, 0, theta, 1)
        lsr.t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta+pi, 1)

        lsr.path_len = self.path_length(lsr.d, self.lc1, self.rc2, lsr.t1, lsr.t2)

        return lsr
    
    def _RSR(self):

        rsr = Params(self.direction['RSR'])

        cline = self.rc2 - self.rc1
        R = np.linalg.norm(cline) / 2
        theta = atan2(cline[1], cline[0]) + acos(0)

        rsr.t1 = transform(self.rc1[0], self.rc1[1], self.r, 0, theta, 1)
        rsr.t2 = transform(self.rc2[0], self.rc2[1], self.r, 0, theta, 1)

        rsr.path_len = self.path_length(rsr.d, self.rc1, self.rc2, rsr.t1, rsr.t2)

        return rsr
    
    def path_length(self, d, c1, c2, t1, t2):
        
        v1 = self.s - c1
        v2 = t1     - c1
        v3 = t2     - t1
        v4 = t2     - c2
        v5 = self.e - c2

        arc1    = calculate_arc_len(v1, v2, d[0], self.r)
        tangent = np.linalg.norm(v3)
        arc2    = calculate_arc_len(v4, v5, d[1], self.r)

        total = arc1 + tangent + arc2
        
        return total
    
    def get_dubins_controls(self, s, dt=1e-4):

        safe = True
        controls = []

        phi1 = self.car.max_phi if s.d[0] == 1 else -self.car.max_phi
        phi2 = 0
        phi3 = self.car.max_phi if s.d[1] == 1 else -self.car.max_phi

        phil = [phi1, phi2, phi3]
        goal = [s.t1, s.t2, self.end_pos[:2]]
        h = [1e-4, 1e-3, 1e-3]
        pos = self.start_pos

        for i in range(len(phil)):
            pos, count, at_goal = self.get_steps(pos, phil[i], goal[i], h[i], dt)
            if at_goal:
                controls.append((phil[i], count, dt))
            else:
                safe = False
                break
        
        return controls, safe

    def get_steps(self, pos, phi, goal, h, dt):

        at_goal = False
        freq = int(1e-2 / dt)
        count = 0
        while True:
            pos = self.car.step(pos, phi, dt)
            count += 1
            
            if count % freq == 0:
                car_state = self.car.get_car_state(pos, phi)
                safe = self.car.env.safe(car_state['vertex'])
                if not safe:
                    break
            
            if same_point(pos[:2], goal, h=h):
                at_goal = True
                break
        
        return pos, count, at_goal


def main():

    obs = [[3, 3.5, 1, 1], [5, 5.5, 1, 1]]
    env = Environment(obs)

    start_pos = [6.5, 2, 1.3*pi]
    end_pos = [3, 7.5, -pi/5]
    car = SimpleCar(env, start_pos, end_pos)

    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    dubins = DubinsPath(car)
    solutions = dubins.find_tangents(start_pos, end_pos)

    tangents = []
    for s in solutions:
        tangents.append([s.t1, s.t2])
    lc = LineCollection(tangents, color=['b','m','b','m'])
    
    lcircle1 = plt.Circle(dubins.lc1, dubins.r, fc='None', ec='k')
    rcircle1 = plt.Circle(dubins.rc1, dubins.r, fc='None', ec='k')
    lcircle2 = plt.Circle(dubins.lc2, dubins.r, fc='None', ec='k')
    rcircle2 = plt.Circle(dubins.rc2, dubins.r, fc='None', ec='k')

    controls = dubins.best_tangent(solutions)
    path = car.get_path(controls)

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
    
    ax = plot_a_car(ax, start_state)
    ax = plot_a_car(ax, end_state)

    _path, = ax.plot([], [], color='g', linewidth=3)
    _car = PatchCollection([])
    ax.add_collection(_car)
    frames = len(path) + 1

    def animate(i):

        xl, yl = [], []
        for j in range(min(i+1, len(path))):
            xl.append(path[j]['pos'][0])
            yl.append(path[j]['pos'][1])
        _path.set_data(xl, yl)

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']
        _car.set_paths(path[min(i, len(path)-1)]['model'])
        _car.set_edgecolor(edgecolor)
        _car.set_facecolor(facecolor)

        return _path, _car

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=1,
                                  repeat=True, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
