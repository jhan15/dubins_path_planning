import numpy as np
from itertools import product


class Grid:
    """ Grid configuration. """

    def __init__(self, env, grid_size=0.5):

        self.env = env
        self.grid_size = grid_size

        self.m = int(self.env.ly / self.grid_size)
        self.n = int(self.env.lx / self.grid_size)
    
    def get_grids(self):
        """ Get all the grids on the map. """

        return [(i, j) for i, j in product(range(self.m), range(self.n))]
    
    def to_grid_id(self, pos):
        """" Convert pose into grid index. """

        pos = np.array(pos)
        grid_id = np.floor(pos / self.grid_size)

        return grid_id
