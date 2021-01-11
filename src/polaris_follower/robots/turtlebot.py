from math import atan2

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from turtlesim.msg import Pose

from polaris_follower.robots.base import Base
from polaris_follower.constants import *


class Turtlebot(Base):
    """
    Robot you control
    """
    scale_factor = 0.5  # to scale the speed as 0.5 * difference

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

    def create_alignment_msg(self, dest_pose_msg):
        """
        :param dest_pose_msg: Destination pose whose direction you want to align
        :return: A Twist message to update the angular velocity
        """
        self_pose = self.get_position_coordinates()
        dest_theta = atan2(dest_pose_msg.y - self_pose.y, dest_pose_msg.x - self_pose.x)

        vel_msg = Twist()
        vel_msg.angular.z = (dest_theta - self_pose.theta)*self.scale_factor
        rospy.loginfo_throttle(LOG_FREQUENCY, str.format("Desired orientation: {}", dest_theta))
        rospy.loginfo_throttle(LOG_FREQUENCY, str.format("Current orientation: {}", self_pose.theta))

        return vel_msg

    def spawn(self, pose_coordinates):
        pass
