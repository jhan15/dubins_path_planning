from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        self.start_pos = [6.2, 2.3, 1.3*pi]
        self.end_pos = [2.9, 7.6, -pi/5]

        self.obs1 = [
            [3, 3.5, 1, 1],
            [5, 5.5, 1, 1]
        ]
        
        self.obs2 = [
            [3, 3.5, 1, 1],
            [4, 4.5, 1, 1],
            [5, 5.5, 1, 1]
        ]

        self.obs3 = [
            [0, 6.1, 6, 0.5],
            [4, 3.1, 6, 0.5]
        ]
