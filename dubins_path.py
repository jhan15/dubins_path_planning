import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from dpp.env.car import SimpleCar
from dpp.env.environment import Environment
from dpp.test_cases.cases import TestCase
from dpp.utils.utils import plot_a_car
from dpp.methods.dubins_path import DubinsPath

from time import time


def main():

    tc = TestCase()

    env = Environment(tc.obs)

    car = SimpleCar(env, tc.start_pos2, tc.end_pos2)

    dubins = DubinsPath(car)

    t = time()
    solutions = dubins.find_tangents(car.start_pos, car.end_pos)
    route, cost, safe = dubins.best_tangent(solutions)

    print('Total time: {}s'.format(round(time()-t, 3)))

    if not safe:
        print('No valid dubins path!')
        return

    path = car.get_path(car.start_pos, route)

    path = path[::5] + [path[-1]]

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
    
    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=5)
    
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
        _carl.set_paths(sub_carl[::4])
        _carl.set_color('m')
        _carl.set_alpha(0.1)

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']
        _car.set_paths(path[min(i, len(path)-1)].model)
        _car.set_edgecolor(edgecolor)
        _car.set_facecolor(facecolor)
        _car.set_zorder(3)

        return _carl, _car

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=1,
                                  repeat=True, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
