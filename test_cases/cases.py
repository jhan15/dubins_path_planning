from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        self.start_pos = [4, 3, -pi/2]
        self.end_pos = [9.5, 7.8, pi]
        # self.end_pos = [13.5, 1, 1.2*pi]
        # self.end_pos = [3.1, 8.5, pi/2]

        self.start_pos2 = [8, 2, -pi/5]
        self.end_pos2 = [9.2, 5.3, 1.4*pi]

        self.obs = [
            [2, 3.5, 4.25, 0.25],
            [2, 1.5, 0.25, 2],
            [6, 1.5, 0.25, 2],
            [2, 6, 0.25, 2.75],
            [2, 6, 4, 0.25],
            [6, 6, 0.25, 2.75],
            [8, 7, 4.25, 0.25],
            [8, 8.5, 4.25, 0.25],
            [8, 7, 0.25, 1.5],
            [12, 3.5, 3, 0.25],
            [10, 0, 0.25, 3.75]
        ]
