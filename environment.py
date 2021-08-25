from obstacle import Obstacle


class Environment:
    """ The map configuration. """

    def __init__(self, obs=None, lx=10, ly=10):

        self.lx  = float(lx)
        self.ly  = float(ly)

        if obs:
            self.obs = [Obstacle(*ob) for ob in obs]
        else:
            self.obs = []
    
    def inbounds(self, vertex, safe_dis=0.1):
        """ Check target within the map bounds. """

        for v in vertex:
            if v[0] < safe_dis:
                return False
            if v[0] > self.lx - safe_dis:
                return False
            if v[1] < safe_dis:
                return False
            if v[1] > self.ly - safe_dis:
                return False
        
        return True

    def obstacle_free(self, vertex):
        """ Check target is obstacle-free or not. """

        for ob in self.obs:
            if not ob.safe(vertex):
                return False
        
        return True

    def safe(self, vertex):
        """ Check target is safe or not. """

        if self.inbounds(vertex) and self.obstacle_free(vertex):
            return True
        
        return False
