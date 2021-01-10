from math import atan2

import rospy
from turtlesim.srv import Spawn

from .constants import *

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Turtle:
    def __init__(self, turtle_name, pose_topic_substr='pose', vel_topic_substr='cmd_vel', pose_msg_type=Pose,
                 vel_msg_type=Twist):
        """
        @param turtle_name: Name of the turtle
        @param pose_topic_substr: substring used to get the pose topic of the turtle
        @param vel_topic_substr: substring used to get velocity topic of the turtle
        @param pose_msg_type: Type of pose message. For turtlesim, it is geometry_msgs.msg.Pose
        @param vel_msg_type: Type of velocity message. For turtlesim, it is turtlesim.msg.Twist

        Please make sure that the following topics exist for your turtle:
            - /<@param turtle_name>/<@param pose_topic_substr>
            - /<@param turtle_name>/<@param vel_topic_substr>
        """
        self.turtle_name = turtle_name
        self.pose_topic = {
            KEY_TOPIC_NAME: str.format('/{}/{}', turtle_name, pose_topic_substr),
            KEY_TOPIC_MSG_TYPE: pose_msg_type
        }
        self.vel_topic = {
            KEY_TOPIC_NAME: str.format('/{}/{}', turtle_name, vel_topic_substr),
            KEY_TOPIC_MSG_TYPE: vel_msg_type
        }

        self.pose = Pose()

    def get_pose_topic(self):
        """
        Returns pose topic name and type
        @return: {topic name = name of the topic, topic type}
        """
        return self.pose_topic

    def get_vel_topic(self):
        """
        Returns velocity topic name and type
        @return: {topic name = name of the topic, topic type}
        """
        return self.vel_topic

    def set_pose(self, pose):
        self.pose = pose

    def get_pose(self):
        return self.pose

    def create_alignment_msg(self, dest_pose_msg):
        """
        @param dest_pose_msg: pose message of the object whose direction the turtle is to be aligned
        @return: a twist message stating the speed of the bot
        """
        dest_theta = atan2(dest_pose_msg.y - self.pose.y, dest_pose_msg.x - self.pose.x)

        vel_msg = Twist()
        vel_msg.angular.z = dest_theta - self.pose.theta
        return vel_msg

    def spawn(self, x, y, theta):
        rospy.wait_for_service('/spawn')
        try:

            spn = rospy.ServiceProxy('/spawn', Spawn)
            spn(x, y, theta, self.turtle_name)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed, spawn")
            rospy.logwarn(e)
