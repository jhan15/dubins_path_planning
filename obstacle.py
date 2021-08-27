from utils.intersection import polygons_overlapping, rectangle_ringsector_intersected


class Obstacle:
    """ Rectangle obstacles. """

    def __init__(self, x, y, w, h, safe_dis=0.1):

        self.x = float(x)
        self.y = float(y)
        self.w = float(w)
        self.h = float(h)

        self.safe_dis = safe_dis

        self.obs = [[self.x-safe_dis,        self.y-safe_dis],
                    [self.x+self.w+safe_dis, self.y-safe_dis],
                    [self.x+self.w+safe_dis, self.y+self.h+safe_dis],
                    [self.x-safe_dis,        self.y+self.h+safe_dis]]
    
    def rectangle_safe(self, rect):
        """ Check a rectangle object is intersected with an obstacle or not. """
        
        return not polygons_overlapping(self.obs, rect)
    
    def ringsector_safe(self, rs):
        """ Check a ringsector object is intersected with an obstacle or not. """

        return not rectangle_ringsector_intersected(self.obs, rs, False)
