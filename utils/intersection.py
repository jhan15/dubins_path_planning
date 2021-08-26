def polygons_intersected(a, b):

    polygons = [a, b]

    for polygon in polygons:

        for i in range(len(polygon)):

            j = (i + 1) % len(polygon)
            p1 = polygon[i]
            p2 = polygon[j]

            normal = [p2[1]-p1[1], p1[0]-p2[0]]

            amin, amax = None, None
            for v in a:
                projected = normal[0]*v[0]+normal[1]*v[1]
                if (amin is None) or (projected < amin):
                    amin = projected

                if (amax is None) or (projected > amax):
                    amax = projected

            bmin, bmax = None, None
            for v in b:
                projected = normal[0]*v[0]+normal[1]*v[1]
                if (bmin is None) or (projected < bmin):
                    bmin = projected

                if (bmax is None) or (projected > bmax):
                    bmax = projected

            if (amax < bmin) or (bmax < amin):
                return False

    return True


def main():

    a = [[1,1], [2,2], [1,3], [0,2]]
    b = [[3,1], [4,1], [4,2], [3,2]]

    print(polygons_intersected(a, b))


if __name__ == '__main__':
    main()
