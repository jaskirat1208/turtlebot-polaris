from time import sleep

import rospy

from polaris_follower.controllers.base_controller import BaseController
from polaris_follower.utils import Point


class Follower(BaseController):
    """
    Given a robot, x, y coordinates of the destination, it reaches points x and y.
    """
    def __init__(self, robot, planner, *args, **kwargs):
        """
        :param robot: The robot which you want to move
        :param planner: Algorithm of path planning to be followed
        """
        self.robot = robot
        self.planner = planner
        super().__init__(*args, **kwargs)

    def simulate(self, dest: Point):
        """
        Given destination point p, moves the robot to the point p(x,y)
        :param dest: Point(x, y)
        """
        sleep(1)  # Sleep so that the robot positions can be updated first
        robot_pose = self.robot.get_position_coordinates()
        start = Point(int(robot_pose.x), int(robot_pose.y))

        # Gets the path from the robot
        robot_path = self.planner.get_path(start, dest)
        rospy.loginfo(str.format("Path suggested by the planner: {}", robot_path))

        for p in robot_path:
            rospy.loginfo(str.format("Heading to point {}", p))
            self.robot.move_to_dest(p.x, p.y)

        rospy.loginfo(str.format("Robot {} has reached its destination", self.robot.object_name))
