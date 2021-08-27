from math import sin, cos, atan2, pi, sqrt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
from time import time


# intersection: cross of edge
# overlapping: overlap of area (could be no cross of edge, i.e. a contains b)


def distance(p1, p2):
    """ Euclidean distance between two points. """
    
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def triangle_area(a, b, c):
    """ Calculate area of a triangle. """

    z1 = b[0]*a[1] - a[0]*b[1]
    z2 = c[0]*b[1] - b[0]*c[1]
    z3 = a[0]*c[1] - c[0]*a[1]

    return abs(z1 + z2 +z3) / 2


def point_line_distance(a, line):
    """ Shortest distance from point to a line. """

    v1 = [line[1][0]-line[0][0], line[1][1]-line[0][1]]
    v2 = [line[0][0]-a[0], line[0][1]-a[1]]

    d = distance(line[0], line[1])

    cross = v1[0]*v2[1] - v2[0]*v1[1]

    return abs(cross) / d


def point_in_rectangle(a, rect):
    """ Check a point inside a rectangle. """

    area = 0

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        p1 = rect[i]
        p2 = rect[j]

        area += triangle_area(p1, p2, a)

    rect_area = 2 * triangle_area(rect[0], rect[1], rect[2])

    if area > rect_area:
        return False
    
    return True


def circle_in_rectangle(circle, rect):
    """ Check a circle inside a rectangle. """

    p = circle[:2]
    r = circle[2]

    if not point_in_rectangle(p, rect):
        return False
    
    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        line = [rect[i], rect[j]]

        if point_line_distance(p, line) < r:
            return False
    
    return True


def separated(line, obj1, obj2):
    """ Two objects are separated by a line. """

    p1, p2 = line
    normal = [p2[1]-p1[1], p1[0]-p2[0]]

    amin, amax = None, None
    for v in obj1:
        projected = normal[0]*v[0]+normal[1]*v[1]
        if (amin is None) or (projected < amin):
            amin = projected

        if (amax is None) or (projected > amax):
            amax = projected
    
    bmin, bmax = None, None
    for v in obj2:
        projected = normal[0]*v[0]+normal[1]*v[1]
        if (bmin is None) or (projected < bmin):
            bmin = projected

        if (bmax is None) or (projected > bmax):
            bmax = projected

    if (amax < bmin) or (bmax < amin):
        return True
    
    return False


def polygons_overlapping(polya, polyb):
    """ Check polygons overlapping. """

    polygons = [polya, polyb]

    for polygon in polygons:

        for i in range(len(polygon)):

            j = (i + 1) % len(polygon)
            edge = [polygon[i], polygon[j]]

            if separated(edge, polya, polyb):
                return False

    return True


def line_rectangle_intersected(line, rect):
    """ Check line segment intersected with a rectangle. """

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        edge = [rect[i], rect[j]]
        
        if not (separated(edge, edge, line) or separated(line, edge, line)):
            return True
    
    return False


def line_rectangle_overlapping(line, rect):
    """ Check line segment overlapping with a rectangle. """

    if separated(line, rect, line):
        return False

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        edge = [rect[i], rect[j]]
        
        if separated(edge, rect, line):
            return False
    
    return True


def line_circle_intersected(line, circle):
    """ Check line segment intersected with a circle. """

    # https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm

    nodes = []

    q, r = circle[:2], circle[2]
    p1, p2 = line

    v1 = [p2[0]-p1[0], p2[1]-p1[1]]
    v2 = [p1[0]-q[0], p1[1]-q[1]]

    a = v1[0]**2 + v1[1]**2
    b = 2 * (v1[0]*v2[0] + v1[1]*v2[1])
    c = (p1[0]**2 + p1[1]**2) + (q[0]**2 + q[1]**2) - 2*(p1[0]*q[0] + p1[1]*q[1]) - r**2

    disc = b**2 - 4 * a * c

    if disc < 0:
        return False, nodes
    
    t1 = (-b + sqrt(disc)) / (2 * a)
    t2 = (-b - sqrt(disc)) / (2 * a)

    if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
        return False, nodes
    
    if 0 <= t1 <= 1:
        nodes.append([p1[0]+t1*v1[0], p1[1]+t1*v1[1]])
    
    if 0 <= t2 <= 1:
        nodes.append([p1[0]+t2*v1[0], p1[1]+t2*v1[1]])
    
    return True, nodes


def rectangle_circle_overlapping(rect, circle):
    """ Check rectangle overlapping with a circle. """

    # if circle's center inside rectangle
    # if rectangle's edge intersected with circle

    p = circle[:2]

    if point_in_rectangle(p, rect):
        return True

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        edge = [rect[i], rect[j]]

        result, _ = line_circle_intersected(edge, circle)

        if result:
            return True
    
    return False


def rectangle_arc_intersected(rect, arc):
    """ Check rectangle intersected with an arc. """

    # arc: [x, y, r, start, end]

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        edge = [rect[i], rect[j]]

        result, nodes = line_circle_intersected(edge, arc[:3])

        if result:
            for node in nodes:
                v = [node[0]-arc[0], node[1]-arc[1]]
                theta = atan2(v[1], v[0]) % (2*pi)

                if arc[3] < arc[4] and arc[3] <= theta <= arc[4]:
                    return True
                
                if arc[3] > arc[4] and not arc[4] < theta <arc[3]:
                    return True
    
    return False


def rectangle_in_ringsector(rect, rs):
    """ Check a rectangle inside a ringsector. """

    # rs: [x, y, rmin, rmax, start, end]
    # if all center-vertex distance within [rmin, rmax] and theta within [start, end]
    
    for p in rect:
        v = [p[0]-rs[0], p[1]-rs[1]]
        d = distance(p, rs[:2])

        if d >= rs[3] or d <= rs[2]:
            return False
        
        theta = atan2(v[1], v[0]) % (2*pi)

        if rs[4] < rs[5] and not rs[4] < theta < rs[5]:
            return False

        if rs[4] > rs[5] and rs[5] <= theta <= rs[4]:
            return False

    return True


def rectangle_ringsector_intersected(rect, rs, edge=True):
    """ Check rectangle intersected with a ringsector. """

    # rs: [x, y, rmin, rmax, sta, end]
    # if rect in ringsector
    # if rect intersected with any arc/edge

    arc1 = rs[:3] + rs[-2:]
    arc2 = rs[:2] + rs[-3:]

    if rectangle_in_ringsector(rect, rs):
        return True

    if rectangle_arc_intersected(rect, arc1):
        return True

    if rectangle_arc_intersected(rect, arc2):
        return True

    if edge:
        p1 = [rs[0] + rs[2]*cos(rs[4]), rs[1] + rs[2]*sin(rs[4])]
        p2 = [rs[0] + rs[3]*cos(rs[4]), rs[1] + rs[3]*sin(rs[4])]
        p3 = [rs[0] + rs[2]*cos(rs[5]), rs[1] + rs[2]*sin(rs[5])]
        p4 = [rs[0] + rs[3]*cos(rs[5]), rs[1] + rs[3]*sin(rs[5])]
        
        if line_rectangle_intersected([p1, p2], rect):
            return True

        if line_rectangle_intersected([p3, p4], rect):
            return True

    return False


def rs_params(rs):
    p1 = [rs[0] + rs[2]*cos(rs[4]), rs[1] + rs[2]*sin(rs[4])]
    p2 = [rs[0] + rs[3]*cos(rs[4]), rs[1] + rs[3]*sin(rs[4])]
    p3 = [rs[0] + rs[2]*cos(rs[5]), rs[1] + rs[2]*sin(rs[5])]
    p4 = [rs[0] + rs[3]*cos(rs[5]), rs[1] + rs[3]*sin(rs[5])]
    return [p1, p2, p3, p4]


def main():

    rs = [1, 1, 1, 2, 7*pi/4, pi/6]
    rect1 = [[3,-1],[4,-1],[4,0],[3,0]]
    rect2 = [[2,1],[4,1],[4,3],[2,3]]

    print(rectangle_ringsector_intersected(rect1, rs))
    print(rectangle_ringsector_intersected(rect2, rs))

    ps = rs_params(rs)
    segs = [ps[:2], ps[-2:]]

    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-2, 5)
    ax.set_ylim(-2, 4)
    ax.set_aspect("equal")

    ax.add_patch(Circle((1,1), 2, fc='None', ec='k'))
    ax.add_patch(Circle((1,1), 1, fc='None', ec='k'))
    
    ax.add_patch(Rectangle((3,-1),1,1,0, fc='None', ec='m', linewidth=2))
    ax.add_patch(Rectangle((2,1),2,2,0, fc='None', ec='m', linewidth=2))
    c = ['r', 'b']
    lc = LineCollection(segs, color=c, linewidth=2)
    ax.add_collection(lc)

    plt.show()


if __name__ == '__main__':
    main()
