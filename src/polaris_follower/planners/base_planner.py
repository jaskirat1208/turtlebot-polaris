from polaris_follower.utils import Point


class BasePlanner:
    def __init__(self):
        """
        Initialize the arena
        """
        pass

    @staticmethod
    def get_path(origin: Point, dest: Point):
        """
        Getting path from origin to destination
        :param origin: Start of the path
        :param dest: End of the path
        :return: List of points
        """
        return [Point(0, 0), Point(1, 1), Point(2, 1), Point(3, 1), Point(2, 2)]
