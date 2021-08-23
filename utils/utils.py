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

    pc = PatchCollection(model, match_original=True)
    ax.add_collection(pc)

    return ax


def calculate_arc_len(vec1, vec2, d, r):
    """ Calculate the arc length of specified rotation direction. """
    
    theta = atan2(vec2[1], vec2[0]) - atan2(vec1[1], vec1[0])

    if theta < 0 and d == 1:
        theta += 2*pi
    elif theta > 0 and d == -1:
        theta -= 2*pi
    
    return abs(theta*r)


def distance(pt1, pt2):
    
    d = sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    return d


def same_point(pt1, pt2, h=1e-3):

    d = distance(pt1, pt2)

    return d < h


def mod_angle(angle):

    return angle % (2*pi)
