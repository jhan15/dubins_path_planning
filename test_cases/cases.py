from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        self.start_pos = [6.5, 2, 1.3*pi]
        self.end_pos = [3, 7.5, -pi/5]

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
            [0, 6.1, 6, 0.3],
            [4, 3.1, 6, 0.3]
        ]
