from shapely.geometry import Polygon


class Obstacle:
    """ Rectangle obstacles. """

    def __init__(self, x, y, w, h):

        self.x = float(x)
        self.y = float(y)
        self.w = float(w)
        self.h = float(h)
    
    def safe(self, car_vertex, safe_dis=0.1):
        """ Check a car state is intersected with an obstacle or not. """

        car = Polygon(car_vertex)
        obs = Polygon([(self.x-safe_dis, self.y-safe_dis),
                       (self.x+self.w+safe_dis, self.y-safe_dis),
                       (self.x+self.w+safe_dis, self.y+self.h+safe_dis),
                       (self.x-safe_dis, self.y+self.h+safe_dis)])
        
        return not car.intersects(obs)
