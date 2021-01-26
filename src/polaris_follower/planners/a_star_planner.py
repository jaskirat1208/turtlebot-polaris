from collections import deque, defaultdict

from polaris_follower.planners import BasePlanner
from polaris_follower.utils import *


class AStarPlanner(BasePlanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.obstacles = set()

    def mark_as_obstacle(self, p: Point):
        """
        Adds this point to the obstacle set
        :param p: Point (x,y)
        """
        self.obstacles.add(p)

    def remove_obstacle(self, p: Point):
        """
        Removes point p from the obstacle set
        :param p: Point (x,y)
        """
        self.obstacles.remove(p)

    def get_path(self, origin: Point, destination: Point):
        return self._a_star(origin, destination)

    # Internal use methods only
    @staticmethod
    def _dist_between(current: Point, neighbor: Point):
        """
        Calculate distance between current and neighbour
        :param current:
        :param neighbor:
        :return:
        """
        return euclidean_distance(current, neighbor)

    @staticmethod
    def _heuristic_estimate(start: Point, goal: Point):
        """
        :param start: Starting point
        :param goal: Goal point
        :return: heuristic estimate of distance between start and end
        """
        return euclidean_distance(start, goal)

    def _neighbor_nodes(self, current: Point):
        neighbours = []
        directions = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]]
        for [dx, dy] in directions:
            candidate = Point(current.x + dx, current.y + dy)
            if candidate not in self.obstacles:
                neighbours.append(candidate)

        return neighbours

    @staticmethod
    def _reconstruct_path(came_from, goal):
        path = deque()
        node = goal
        path.appendleft(node)
        while node in came_from:
            node = came_from[node]
            path.appendleft(node)
        return path

    @staticmethod
    def _get_lowest(open_set, f_score):
        lowest = float("inf")
        lowest_node = None
        for node in open_set:
            if f_score[node] < lowest:
                lowest = f_score[node]
                lowest_node = node
        return lowest_node

    def _a_star(self, start: Point, goal: Point):
        came_from = {}
        open_set = set()
        open_set.add(start)
        closed_set = set()
        g_score = defaultdict()
        f_score = defaultdict()
        g_score[start] = 0
        f_score[start] = g_score[start] + self._heuristic_estimate(start, goal)
        while len(open_set) != 0:
            current = self._get_lowest(open_set, f_score)
            if current == goal:
                return self._reconstruct_path(came_from, goal)
            open_set.remove(current)
            closed_set.add(current)
            for neighbor in self._neighbor_nodes(current):
                tentative_g_score = g_score[current] + self._dist_between(current, neighbor)
                if neighbor in closed_set and tentative_g_score >= g_score[neighbor]:
                    continue
                if neighbor not in closed_set or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self._heuristic_estimate(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)
        raise Exception(str.format("Path cannot be found from {}->{}", start, goal))
