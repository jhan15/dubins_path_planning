from math import cos, sin, pi


class Circle:

    def __init__(self, x, y, r):

        self.x = x
        self.y = y
        self.r = r

    def point_on_circle(self, theta):

        x = self.x + self.r * cos(theta)
        y = self.y + self.r * sin(theta)

        return x, y


class Arc(Circle):

    def __init__(self, x, y, r, start, end):
        
        super().__init__(x, y, r)

        self.start = start
        self.end = end
    
    def point_on_arc(self, theta):

        theta = theta % (2*pi)

        if not self.start<=theta<=self.end or self.end<theta<self.start:
            print('Out of arc!')
            return

        return self.point_on_circle(theta)
