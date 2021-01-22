from math import atan2

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from turtlesim.msg import Pose

from polaris_follower.robots.base import BaseRobot


class Turtlebot(BaseRobot):
    max_speed = 1
    """
    Turtlebot: could be in a gazebo simulator or a real turtlebot
    """
    def __init__(self, turtle_name, pose_topic_substr='odom', vel_topic_substr='cmd_vel', pose_msg_type=Odometry,
                 vel_msg_type=Twist, namespace='turtlebot_align'):
        super().__init__(turtle_name, pose_topic_substr, vel_topic_substr, pose_msg_type, vel_msg_type, namespace)

    def get_position_coordinates(self):
        """
        :return: returns a geometry_msgs Pose object
        """
        position = self.pose.pose.pose.position
        orientation = self.pose.pose.pose.orientation
        x = position.x
        y = position.y

        # Angle of rotation wrt z axis
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return Pose(x, y, yaw, 0, 0)

    def _get_rotation_angle(self, x, y):
        """
        :param x: x coordinate of destination
        :param y: y coordinate of destination
        :return:
        """
        self_pose = self.get_position_coordinates()
        dest_theta = atan2(y - self_pose.y, x - self_pose.x)

        return dest_theta - self_pose.theta

    def spawn(self, pose_coordinates):
        pass
