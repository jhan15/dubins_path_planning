from math import cos, sin, atan2, pi, sqrt
from matplotlib.collections import PatchCollection
import numpy as np


def transform(x, y, w, l, theta, id):
    """ Coordinates transform. """

    if id == 1:
        x_ = x + w*cos(theta) - l*sin(theta)
        y_ = y + w*sin(theta) + l*cos(theta)
    if id == 2:
        x_ = x + w*cos(theta) + l*sin(theta)
        y_ = y + w*sin(theta) - l*cos(theta)
    if id == 3:
        x_ = x - w*cos(theta) - l*sin(theta)
        y_ = y - w*sin(theta) + l*cos(theta)
    if id == 4:    
        x_ = x - w*cos(theta) + l*sin(theta)
        y_ = y - w*sin(theta) - l*cos(theta)
    
    return np.array([x_, y_])


def plot_a_car(ax, model):
    """ Plot a car model. """

    pc = PatchCollection(model, match_original=True, zorder=2)
    ax.add_collection(pc)

    return ax


def arc_length(pos1, pos2, r):
    """ Calculate the arc and chord length. """
    
    delta_theta = pos2[2] - pos1[2]

    if delta_theta != 0:
        arc = abs(delta_theta*r)
        chord = abs(2*r*sin(delta_theta/2))
    else:
        arc = distance(pos1[:2], pos2[:2])
        chord = arc
    
    return arc, chord


def directional_theta(vec1, vec2, d):
    """ Calculate the directional theta change. """
    
    theta = atan2(vec2[1], vec2[0]) - atan2(vec1[1], vec1[0])

    if theta < 0 and d == 1:
        theta += 2*pi
    elif theta > 0 and d == -1:
        theta -= 2*pi
    
    return theta


def distance(pt1, pt2):
    """ Distance of two points. """
    
    d = sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    return d


def same_point(pt1, pt2, h=1e-2):
    """ Check two points are same within a samll error. """

    d = distance(pt1, pt2)

    return d < h


def round_theta(theta, thetas):
    """ Round theta to closest discretized value. """

    return min(thetas, key=lambda x: abs(x-theta) % (2*pi))


def get_discretized_thetas(unit_theta):
    """ Get all discretized theta values by unit value. """

    thetas = [0]

    while True:
        theta = thetas[-1] + unit_theta
        if theta > (2*pi - unit_theta):
            break
        
        thetas.append(theta)
    
    return thetas
