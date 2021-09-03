from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        self.start_pos = [4.6, 2.4, 0]
        self.end_pos = [1.6, 8, -pi/2]

        self.start_pos2 = [4, 4, 0]
        self.end_pos2 = [4, 8, 1.2*pi]

        self.obs = [
            [2, 3, 6, 0.1],
            [2, 3, 0.1, 1.5],
            [4.3, 0, 0.1, 1.8],
            [6.5, 1.5, 0.1, 1.5],
            [0, 6, 3.5, 0.1],
            [5, 6, 5, 0.1]
        ]
