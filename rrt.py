import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
import matplotlib.animation as animation

from dpp.env.car import SimpleCar
from dpp.env.environment import Environment
from dpp.test_cases.cases import TestCase
from dpp.utils.utils import plot_a_car
from dpp.methods.rrt import RRT

from time import time


def main():

    tc = TestCase()

    env = Environment(tc.obs)

    car = SimpleCar(env, tc.start_pos, tc.end_pos)

    rrt = RRT(car)

    t = time()

    path, nodes = rrt.search_path()

    print('Total time: {}s'.format(round(time()-t, 3)))

    path = path[::5] + [path[-1]]

    nodes = nodes[1:]
    branches = []
    nodex, nodey = [], []
    
    for node in nodes:
        branches.append(node.branch)
        nodex.append(node.pos[0])
        nodey.append(node.pos[1])
    
    xl, yl = [], []
    carl = []
    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])

    end_state = car.get_car_state(car.end_pos)
    start_state = car.get_car_state(car.start_pos)

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
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)

    _branches = LineCollection([], color='b', alpha=0.8, linewidth=1)
    ax.add_collection(_branches)
    _nodes, = ax.plot([], [], 'ro', markersize=4)

    _path, = ax.plot([], [], color='lime', linewidth=2)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='whitesmoke', linewidth=2)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(branches) + len(path) + 1

    def init():
        _branches.set_paths([])
        _nodes.set_data([], [])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])

        return _branches, _nodes, _path, _carl, _path1, _car

    def animate(i):

        if i < len(branches):
            _branches.set_paths(branches[:i+1])
            _nodes.set_data(nodex[:i+1], nodey[:i+1])
        
        else:
            _branches.set_paths(branches)
            _nodes.set_data(nodex, nodey)

            j = i - len(branches)

            _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])

            sub_carl = carl[:min(j+1, len(path))]
            _carl.set_paths(sub_carl[::4])
            _carl.set_color('m')
            _carl.set_alpha(0.1)
            _carl.set_zorder(3)

            _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
            _path1.set_zorder(3)

            edgecolor = ['k']*5 + ['r']
            facecolor = ['y'] + ['k']*4 + ['r']
            _car.set_paths(path[min(j, len(path)-1)].model)
            _car.set_edgecolor(edgecolor)
            _car.set_facecolor(facecolor)
            _car.set_zorder(3)

        return _branches, _nodes, _path, _carl, _path1, _car

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, 
                                  interval=1, repeat=True, blit=True)

    plt.show()


if __name__ == '__main__':
    main()
