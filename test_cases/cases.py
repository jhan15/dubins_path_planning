from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        # self.start_pos = [6.2, 1.3, 1.2*pi]
        # self.start_pos = [1.5, 2.6, pi/6]
        self.start_pos = [1.5, 1, pi/6]
        # self.end_pos = [2, 1.2, 1.2*pi]
        # self.end_pos = [5, 8.1, 1.2*pi]
        self.end_pos = [14, 7.8, pi]
        # self.end_pos = [9.2, 1.2, 1.2*pi]
        # self.end_pos = [11.2, 1.2, 1.2*pi]
        # self.end_pos = [18.2, 1.2, 1.2*pi]
        # self.end_pos = [13, 8, pi/5]
        # self.end_pos = [18.2, 4.5, pi]

        self.start_pos2 = [13.5, 2, -pi/5]
        self.end_pos2 = [13, 5.2, 1.3*pi]

        self.obs = [
            [2, 7, 4.25, 0.25],
            [2, 5, 0.25, 2],
            [6, 5, 0.25, 2],
            [0, 2, 3.5, 0.25],
            [10, 7, 0.25, 1.5],
            [10, 7, 6, 0.25],
            [14, 3, 6, 0.25],
            [6, 0, 0.25, 3],
            [8, 0, 0.25, 3],
            [10, 0, 0.25, 5]
        ]
