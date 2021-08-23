from obstacle import Obstacle


class Environment:
    """ The map configuration. """

    def __init__(self, obs=None):

        self.lx  = float(10)
        self.ly  = float(10)

        if obs:
            self.obs = [Obstacle(*ob) for ob in obs]
        else:
            self.obs = []
    
    def inbounds(self, car_vertex, safe_dis=0.1):
        """ Check car within the map bounds. """

        for v in car_vertex:
            if v[0] < safe_dis:
                return False
            if v[0] > self.lx - safe_dis:
                return False
            if v[1] < safe_dis:
                return False
            if v[1] > self.ly - safe_dis:
                return False
        
        return True

    def obstacle_free(self, car_vertex):
        """ Check car is obstacle-free or not. """

        for ob in self.obs:
            if not ob.safe(car_vertex):
                return False
        
        return True

    def safe(self, car_vertex):
        """ Check car is safe or not. """

        if self.inbounds(car_vertex) and self.obstacle_free(car_vertex):
            return True
        
        return False
