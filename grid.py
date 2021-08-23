import numpy as np
from itertools import product


class Grid:
    """ Grid configuration. """

    def __init__(self, env, grid_size=0.5):

        self.env = env
        self.grid_size = grid_size
    
    def get_grids(self):
        """ Get all the grids on the map. """

        m = int(self.env.ly / self.grid_size)
        n = int(self.env.lx / self.grid_size)

        return [(i, j) for i, j in product(range(m), range(n))]
    
    def grid_id(self, x):
        """" Convert pose into grid index. """

        x = np.array(x)
        x_grid = np.floor(x / self.grid_size)

        return x_grid
