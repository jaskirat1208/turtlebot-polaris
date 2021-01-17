from time import sleep

import rospy

from polaris_follower.controllers.base_controller import BaseController
from polaris_follower.utils import Point


class Follower(BaseController):
    """
    Given a robot, x, y coordinates of the destination, it reaches points x and y.
    """
    def __init__(self, robot, planner,  *args, **kwargs):
        """
        :param robot: The robot which you want to move
        :param planner: Means of getting path to be followed
        """
        self.robot = robot
        self.planner = planner
        super().__init__(*args, **kwargs)

    def simulate(self):
        """
        Given x, y coordinates of the destination, moves the robot to the point (x,y)
        :param x: X coordinate of destination
        :param y: Y coordinate of destination
        :return:
        """
        sleep(1)    # Sleep so that the robot positions can be updated first
        robot_path = self.planner.get_path(Point(0, 0), Point(0, 1))
        for p in robot_path:
            rospy.loginfo(str.format("Heading to point {}", p))
            self.robot.move_to_dest(p.x, p.y)

        rospy.loginfo(str.format("Robot {} has reached its destination", self.robot.object_name))
