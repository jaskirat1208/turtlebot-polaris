from math import sqrt


class Point:
    """
    A point in a cartesan plane
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self):
        return str.format("({}, {})", self.x, self.y)

    def __eq__(self, other):
        if not isinstance(other, type(self)):
            return NotImplemented

        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return self.__str__()


def euclidean_distance(p1: Point, p2: Point):
    """
    :param p1: (x1,y1)
    :param p2: (x2,y2)
    :return: Manhattan distance between points p1 and p2
    """
    return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def get_point_from_user_input():
    """
    Scans user input for x and y coordinates and creates a point object
    :return: Point(x, y)
    """
    x = int(input("Enter x coordinate of destination"))
    y = int(input("Enter y coordinate of destination"))

    return Point(x, y)
