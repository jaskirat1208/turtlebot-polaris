from polaris_follower.controllers.base_controller import BaseController


class Follower(BaseController):
    """
    Given a robot, x, y coordinates of the destination, it reaches points x and y.
    """
    def __init__(self, robot, *args, **kwargs):
        """
        :param robot: The robot which you want to move
        """
        self.robot = robot
        super().__init__(*args, **kwargs)

    def simulate(self, x, y):
        """
        Given x, y coordinates of the destination, moves the robot to the point (x,y)
        :param x: X coordinate of destination
        :param y: Y coordinate of destination
        :return:
        """
        self.robot.move_to_dest(x, y)
