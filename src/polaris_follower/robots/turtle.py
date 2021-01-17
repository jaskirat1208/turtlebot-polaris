# Basic imports
from math import atan2, sqrt

# ROS imports
import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

# Local imports
from polaris_follower.robots.base import BaseRobot


class Turtle(BaseRobot):
    def __init__(self, turtle_name, pose_topic_substr='pose', vel_topic_substr='cmd_vel', pose_msg_type=Pose,
                 vel_msg_type=Twist, namespace='turtlesim_align'):
        """
        @param turtle_name: Name of the turtle
        @param pose_topic_substr: substring used to get the pose topic of the turtle
        @param vel_topic_substr: substring used to get velocity topic of the turtle
        @param pose_msg_type: Type of pose message. For turtlesim, it is geometry_msgs.msg.Pose
        @param vel_msg_type: Type of velocity message. For turtlesim, it is turtlesim.msg.Twist
        @param namespace: Namespace of the turtle, as specified in the launch file

        Please make sure that the following topics exist for your turtle:
            - /<@param turtle_name>/<@param pose_topic_substr>
            - /<@param turtle_name>/<@param vel_topic_substr>
        """
        super().__init__(turtle_name, pose_topic_substr, vel_topic_substr, pose_msg_type, vel_msg_type, namespace)

    def _get_rotation_angle(self, x, y):
        return atan2(y - self.pose.y, x - self.pose.x) - self.pose.theta

    def _get_disp_vector(self, x, y):
        return Pose(x - self.pose.x, y - self.pose.y, 0, 0, 0)

    def get_position_coordinates(self):
        """
        :return: returns a turtlesim.msg -> Pose object
        """
        return self.pose

    def spawn(self, pose_coordinates):
        """
        :param pose_coordinates: turtlesim.msg -> Pose object
        """
        rospy.wait_for_service(self.spawn_service)
        try:
            spn = rospy.ServiceProxy(self.spawn_service, Spawn)
            spn(pose_coordinates.x, pose_coordinates.y, pose_coordinates.theta, self.object_name)
            rospy.loginfo('Turtle successfully spawned')
        except rospy.ServiceException as e:
            rospy.logwarn('Service call failed, spawn')
            rospy.logwarn(e)
