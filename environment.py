from obstacle import Obstacle
from utils.intersection import rectangle_ringsector_intersected


class Environment:
    """ The map configuration. """

    def __init__(self, obs=None, lx=10, ly=10):

        self.lx  = float(lx)
        self.ly  = float(ly)

        if obs:
            self.obs = [Obstacle(*ob) for ob in obs]
        else:
            self.obs = []
    
    def rectangle_inbounds(self, rect, safe_dis=0.05):
        """ Check rectangle target within the map bounds. """

        for v in rect:
            if v[0] < safe_dis:
                return False
            if v[0] > self.lx - safe_dis:
                return False
            if v[1] < safe_dis:
                return False
            if v[1] > self.ly - safe_dis:
                return False
        
        return True
    
    def ringsector_inbounds(self, rs, safe_dis=0.05):
        """ Check ringsector target within the map bounds. """

        rect = [[0+safe_dis,        0+safe_dis],
                [self.lx-safe_dis,  0+safe_dis],
                [self.lx-safe_dis,  self.ly-safe_dis],
                [0+safe_dis,        self.ly-safe_dis]]
        
        return not rectangle_ringsector_intersected(rect, rs, False)

    def rectangle_obstacle_free(self, rect):
        """ Check rectangle target is obstacle-free or not. """

        for ob in self.obs:
            if not ob.rectangle_safe(rect):
                return False
        
        return True
    
    def ringsector_obstacle_free(self, rs):
        """ Check ringsector target is obstacle-free or not. """

        for ob in self.obs:
            if not ob.ringsector_safe(rs):
                return False
        
        return True

    def rectangle_safe(self, rect):
        """ Check rectangle target is safe or not. """

        if self.rectangle_inbounds(rect) and self.rectangle_obstacle_free(rect):
            return True
        
        return False
    
    def ringsector_safe(self, rs):
        """ Check ringsector target is safe or not. """

        if self.ringsector_inbounds(rs) and self.ringsector_obstacle_free(rs):
            return True
        
        return False
