# Basic imports
from math import atan2

# ROS imports
import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

# Local imports
from polaris_follower.objects.base import Base


class Turtle(Base):
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

    def create_alignment_msg(self, dest_pose_msg):
        """
        @param dest_pose_msg: pose message of the object whose direction the turtle is to be aligned
        @return: a twist message stating the speed of the bot
        """
        dest_theta = atan2(dest_pose_msg.y - self.pose.y, dest_pose_msg.x - self.pose.x)

        vel_msg = Twist()
        vel_msg.angular.z = dest_theta - self.pose.theta
        return vel_msg

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
