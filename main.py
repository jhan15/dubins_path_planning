import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from car import SimpleCar
from environment import Environment
from test_cases.cases import TestCase
from rrt import RRT
from utils.utils import plot_a_car


def main():

    # test cases
    tc = TestCase()

    # map w/ obstacles
    env = Environment(tc.obs3)

    # car w/ initial and target poses
    car = SimpleCar(env, tc.start_pos, tc.end_pos)

    # RRT + Dubins path
    rrt = RRT(car)

    # pathfinding
    controls = rrt.search_path()
    path = car.get_path(car.start_pos, controls)

    xl, yl = [], []
    carl = []
    for i in range(len(path)):
        xl.append(path[i]['pos'][0])
        yl.append(path[i]['pos'][1])
        carl.append(path[i]['model'][0])

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
    
    ax = plot_a_car(ax, end_state['model'])

    _path, = ax.plot([], [], color='lime', linewidth=1)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(path) + 1

    def animate(i):

        _path.set_data(xl[min(i, len(path)-1):], yl[min(i, len(path)-1):])

        sub_carl = carl[:min(i+1, len(path))]
        _carl.set_paths(sub_carl[::10])
        _carl.set_edgecolor('m')
        _carl.set_facecolor('None')
        _carl.set_alpha(0.2)

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']
        _car.set_paths(path[min(i, len(path)-1)]['model'])
        _car.set_edgecolor(edgecolor)
        _car.set_facecolor(facecolor)
        _car.set_zorder(3)

        return _path, _carl, _car

    ani = animation.FuncAnimation(fig, animate, frames=frames, interval=1,
                                  repeat=False, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
