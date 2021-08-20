from obstacle import Obstacle


class Environment:

    def __init__(self, obs=None):

        self.lx  = float(10)
        self.ly  = float(10)

        if obs:
            self.obs = [Obstacle(*ob) for ob in obs]
        else:
            self.initialize_obs()
    
    def initialize_obs(self):

        ob1 = Obstacle(5, 4, 0.5, 6)
        ob2 = Obstacle(10, 0, 0.5, 6)
        ob3 = Obstacle(3, 3.5, 1, 1)
        ob4 = Obstacle(5, 5.5, 1, 1)
        ob5 = Obstacle(4, 4.5, 1, 1)
        self.obs = [ob3, ob4, ob5]
    
    def inbounds(self, car_vertex, safe_dis=0.1):

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

        for ob in self.obs:
            if not ob.safe(car_vertex):
                return False
        
        return True

    def safe(self, car_vertex):

        if self.inbounds(car_vertex) and self.obstacle_free(car_vertex):
            return True
        
        return False
