from math import sin, cos, atan2, pi, sqrt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from time import time


def triangle_area(a, b, c):

    z1 = b[0]*a[1] - a[0]*b[1]
    z2 = c[0]*b[1] - b[0]*c[1]
    z3 = a[0]*c[1] - c[0]*a[1]

    return abs(z1 + z2 +z3) / 2


def point_line_distance(a, line):

    a = np.array(a)
    line = np.array(line)

    v1 = line[1] - line[0]
    v2 = line[0] - a

    l = np.linalg.norm(v1)

    return abs(np.cross(v1, v2)) / l


def point_in_rectangle(a, rect):

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

    p1, p2 = line
    obj1 = np.array(obj1)
    obj2 = np.array(obj2)

    normal = np.array([p2[1]-p1[1], p1[0]-p2[0]])

    dotl = []
    for p in obj1:
        dotl.append(normal @ p)
    amin, amax = min(dotl), max(dotl)

    dotl = []
    for p in obj2:
        dotl.append(normal @ p)
    bmin, bmax = min(dotl), max(dotl)

    if (amax < bmin) or (bmax < amin):
        return True
    
    return False


def polygons_intersected(polya, polyb):

    polygons = np.array([polya, polyb])

    for polygon in polygons:

        for i in range(len(polygon)):

            j = (i + 1) % len(polygon)
            edge = [polygon[i], polygon[j]]

            if separated(edge, polya, polyb):
                return False

    return True


def line_rectangle_intersected(line, rect):

    line = np.array(line)
    rect = np.array(rect)

    if separated(line, rect, line):
        return False

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        edge = [rect[i], rect[j]]
        
        if separated(edge, rect, line):
            return False
    
    return True


def line_circle_intersected(line, circle):

    # https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm

    nodes = []

    q, r = np.array(circle[:2]), circle[2]
    p1, p2 = np.array(line[0]), np.array(line[1])

    v1 = p2 - p1
    v2 = p1 - q
    
    a = v1 @ v1
    b = 2 * v1 @ v2
    c = p1 @ p1 + q @ q - 2 * p1 @ q - r**2

    disc = b**2 - 4 * a * c

    if disc < 0:
        return False, nodes
    
    t1 = (-b + sqrt(disc)) / (2 * a)
    t2 = (-b - sqrt(disc)) / (2 * a)

    if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
        return False, nodes
    
    if 0 <= t1 <= 1:
        nodes.append(p1 + t1*v1)
    
    if 0 <= t2 <= 1:
        nodes.append(p1 + t2*v1)
    
    return True, nodes


def rectangle_circle_intersected(rect, circle):

    # if circle's center inside rectangle
    # if rectangle's edge intersected with circle

    p = circle[:2]

    if point_in_rectangle(p, rect):
        return True

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        line = [rect[i], rect[j]]

        result, _ = line_circle_intersected(line, circle)

        if result:
            return True
    
    return False


def rectangle_arc_intersected(rect, arc):

    # arc: [x, y, r, start, end]

    circle = arc[:3]
    q = np.array(arc[:2])
    start, end = arc[3], arc[4]

    for i in range(len(rect)):

        j = (i + 1) % len(rect)
        line = [rect[i], rect[j]]

        result, nodes = line_circle_intersected(line, circle)

        if result:
            for node in nodes:
                v = node - q
                theta = atan2(v[1], v[0]) % (2*pi)
                
                if start <= theta <= end or not end < theta < start:
                    return True
    
    return False


def rectangle_in_ringsector(rect, rs):

    # rs: [x, y, rmin, rmax, start, end]
    # if all center-vertex distance within [rmin, rmax] and theta within [start, end]

    q = np.array(rs[:2])
    rmin, rmax = rs[2], rs[3]
    start, end = rs[4], rs[5]
    rect = np.array(rect)

    print(start, end)
    
    for p in rect:
        v = p - q

        d = np.linalg.norm(v)
        if d >= rmax or d <= rmin:
            return False
        
        theta = atan2(v[1], v[0]) % (2*pi)
        if not start < theta < end or end <= theta <= start:
            return False

    return True


def rectangle_ringsector_intersected(rect, rs):

    # rs: [x, y, rmin, rmax, start, end]
    # if rect in ringsector
    # if rect intersected with any arc

    arc1 = rs[:3] + rs[-2:]
    arc2 = rs[:2] + rs[-3:]

    x, y = rs[0], rs[1]
    rmin, rmax = rs[2], rs[3]
    start, end = rs[4], rs[5]

    p1 = [x+rmin*cos(start), y+rmin*sin(start)]
    p2 = [x+rmax*cos(start), y+rmax*sin(start)]
    p3 = [x+rmin*cos(end), y+rmin*sin(end)]
    p4 = [x+rmax*cos(end), y+rmax*sin(end)]

    if line_rectangle_intersected([p1,p2], rect):
        return True
    
    if line_rectangle_intersected([p3,p4], rect):
        return True

    if rectangle_in_ringsector(rect, rs):
        return True

    if rectangle_arc_intersected(rect, arc1):
        return True
    
    if rectangle_arc_intersected(rect, arc2):
        return True

    return False


def main():

    rect = [[4,2],[5,2],[5,4],[4,4]]
    line1 = [[3.5,1.5],[5.5,3]]
    line2 = [[4.5,2.5],[4.8,3.6]]
    line3 = [[4.5,2.5],[3.5,5]]
    line4 = [[3.5,3.8],[4.5,4.5]]

    # print(line_rectangle_intersected(line1, rect))
    # print(line_rectangle_intersected(line2, rect))
    # print(line_rectangle_intersected(line3, rect))
    # print(line_rectangle_intersected(line4, rect))


    rs = [1,1,1,2,pi/4,pi/2]
    rect1 = [[2.5,1], [3.5,2], [2.5,3], [1.5,2]]
    rect2 = [[1.2,2.2], [1.5,2.2], [1.5,2.5], [1.2,2.5]]
    rect3 = [[1.4,2], [2,2], [2,2.8], [1.4,2.8]]
    rect4 = [[2.5,1.5], [3,2], [2.5,2.5], [2,2]]
    rect5 = [[2.2,1.8], [2.4,2], [2.2,2.2], [2,2]]

    print(polygons_intersected(rect1, rect2))
    print(polygons_intersected(rect1, rect3))
    print(polygons_intersected(rect2, rect3))
    print(polygons_intersected(rect1, rect4))

    print(polygons_intersected(rect, rect1))
    print(polygons_intersected(rect, rect2))
    print(polygons_intersected(rect, rect3))

    # print(rectangle_in_ringsector(rect1, rs))
    # print(rectangle_ringsector_intersected(rect1, rs))

    # print(rectangle_in_ringsector(rect2, rs))
    # print(rectangle_ringsector_intersected(rect2, rs))

    # print(rectangle_in_ringsector(rect3, rs))
    # print(rectangle_ringsector_intersected(rect3, rs))

    # print(rectangle_in_ringsector(rect4, rs))
    # print(rectangle_ringsector_intersected(rect4, rs))

    # print(rectangle_in_ringsector(rect5, rs))
    # print(rectangle_ringsector_intersected(rect5, rs))

    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-2, 6)
    ax.set_ylim(-2, 6)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    ax.plot([1,1+sqrt(2)], [1,1+sqrt(2)])
    ax.plot([1,1], [1,3])
    
    ax.add_patch(Circle((1,1), 2, fc='y', ec='k'))
    ax.add_patch(Circle((1,1), 1, fc='w', ec='k'))
    ax.add_patch(Rectangle((2.5,1), sqrt(2), sqrt(2), 45, fc='None', ec='k'))
    ax.add_patch(Rectangle((1.2,2.2), 0.3, 0.3, 0, fc='None', ec='k'))
    ax.add_patch(Rectangle((1.4,2), 0.6, 0.8, 0, fc='None', ec='k'))
    ax.add_patch(Rectangle((2.5,1.5), sqrt(2)/2, sqrt(2)/2, 45, fc='None', ec='k'))
    ax.add_patch(Rectangle((2.2,1.8), sqrt(0.08), sqrt(0.08), 45, fc='None', ec='k'))

    ax.add_patch(Rectangle((4,2), 1, 2, 0, fc='None', ec='k'))
    ax.plot([3.5,5.5], [1.5,3])
    ax.plot([4.5,4.8], [2.5,3.6])
    ax.plot([4.5,3.5], [2.5,5])
    ax.plot([3.5,4.5], [3.8,4.5])

    plt.show()


if __name__ == '__main__':
    main()
