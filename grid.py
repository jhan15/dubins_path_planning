import numpy as np
from math import sqrt
from itertools import product


class Grid:
    """ Grid configuration. """

    def __init__(self, env, cell_size=0.25):

        self.env = env
        self.cell_size = cell_size

        self.m = int(self.env.ly / self.cell_size)
        self.n = int(self.env.lx / self.cell_size)

        self.cell_dia = sqrt(2*self.cell_size**2)
    
    # def get_grids(self):
    #     """ Get all the grids on the map. """

    #     return [(i, j) for i, j in product(range(self.m), range(self.n))]
    
    def to_cell_id(self, pt):
        """" Convert point into grid index. """

        pt = np.array(pt)
        cell_id = np.floor(pt / self.cell_size)

        return cell_id
