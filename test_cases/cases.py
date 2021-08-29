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
            [2.5, 5, 4, 0.25]
        ]

        self.obs4 = [
            [0, 6, 5.5, 0.25],
            [5.5, 6, 0.25, 2.5]
        ]

        self.obs5 = [
            [0, 6, 6, 0.25],
            [4, 3, 6, 0.25]
        ]

        self.obs6 = [
            [0, 6, 6, 0.25],
            [6, 6, 0.25, 2],
            [4, 3, 6, 0.25],
            [4, 1.5, 0.25, 1.5]
        ]
