from math import sqrt

import rospy
from turtlesim.msg import Pose

from polaris_follower.constants import *


class BaseRobot:
    scale_factor = 0.5  # to scale the speed as 0.5 * difference

    def __init__(self, turtle_name, pose_topic_substr, vel_topic_substr, pose_msg_type,
                 vel_msg_type, namespace='tbot_align'):
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
        rospy.loginfo(str.format("Initializing {} of type: {}", turtle_name, self.__class__))
        self.object_name = turtle_name
        self.object_ns = namespace

        # pose and vel topics
        self.pose_topic = {
            KEY_TOPIC_NAME: str.format('/{}/{}/{}', self.object_ns, self.object_name, pose_topic_substr),
            KEY_TOPIC_MSG_TYPE: pose_msg_type
        }
        self.vel_topic = {
            KEY_TOPIC_NAME: str.format('/{}/{}/{}', self.object_ns, self.object_name, vel_topic_substr),
            KEY_TOPIC_MSG_TYPE: vel_msg_type
        }
        rospy.loginfo("Pose topic:")
        rospy.loginfo(self.pose_topic)

        rospy.loginfo("Vel Topic:")
        rospy.loginfo(self.vel_topic)

        # Spawn service
        self.spawn_service = str.format('/{}/spawn', self.object_ns)
        rospy.loginfo("Spawn service:")
        rospy.loginfo(self.spawn_service)

        # Creating publisher and subscriber for the robot
        self.vel_pub = rospy.Publisher(
            self.vel_topic[KEY_TOPIC_NAME], self.vel_topic[KEY_TOPIC_MSG_TYPE], queue_size=QUEUE_SIZE)
        self.pose_sub = rospy.Subscriber(
            self.pose_topic[KEY_TOPIC_NAME], self.pose_topic[KEY_TOPIC_MSG_TYPE], self.pose_cb)

        self.pose = pose_msg_type()

    def pose_cb(self, pose_data):
        """
        A callback function to pose subscriber. Updates the position of the robot
        :param pose_data: Message fetched by the subscriber
        """
        rospy.loginfo_throttle(LOG_FREQUENCY, str.format("Updating pose data for {}", self.object_name))
        self.set_pose(pose_data)

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

    def get_position_coordinates(self):
        """
        Returns the position coordinates (x,y, theta)
        :return:
        """
        return Pose()

    def _get_rotation_angle(self, x, y):
        return 0

    def _get_distance_from_xy(self, x, y):
        self_pose = self.get_position_coordinates()
        return sqrt((x - self_pose.x)**2 + (y - self_pose.y)**2)

    def _translate(self, x, y):
        """
        Moves towards (x, y)
        :param x: x coordinate of destination
        :param y: y coordinate of destination
        :return:
        """
        rospy.loginfo_once("Translation begins")

        vel_msg = self.vel_topic[KEY_TOPIC_MSG_TYPE]()
        dist = self._get_distance_from_xy(x, y)
        while dist > ANGULAR_DIST_THRESHOLD:
            vel_msg.linear.x = min(dist, 2) * self.scale_factor
            vel_msg.angular.z = self._get_rotation_angle(x, y) * self.scale_factor
            rospy.loginfo_throttle(LOG_FREQUENCY, str.format("Moving to ({}, {}). {} units away", x, y, dist))

            self.vel_pub.publish(vel_msg)
            dist = self._get_distance_from_xy(x, y)

        rospy.loginfo_once(str.format("Translation complete. {} is now at ({}, {})", self.object_name, x, y))

    def _rotate(self, x, y):
        """
        Rotates in the direction of x,y
        :param x: x coordinate of destination
        :param y: y coordinate of destination
        :return:
        """
        rospy.loginfo_once("Rotation begins")

        vel_msg = self.vel_topic[KEY_TOPIC_MSG_TYPE]()
        theta = self._get_rotation_angle(x, y)
        while abs(theta) > ANGULAR_DIST_THRESHOLD:
            vel_msg.angular.z = theta * self.scale_factor
            self.vel_pub.publish(vel_msg)
            theta = self._get_rotation_angle(x, y)

        rospy.loginfo_once("Rotation complete")

    def align_with_dest(self, dest_obj):
        """
        :param dest_obj: Destination robot along which it self-aligns
        """
        while not rospy.is_shutdown():
            dest_pose_msg = dest_obj.get_position_coordinates()
            self._rotate(dest_pose_msg.x, dest_pose_msg.y)

    def move_to_dest(self, x, y):
        """
        Given x, y, moves the robot to the point (x,y)
        :param x: x coordinate
        :param y: y coordinate
        :return:
        """
        self._rotate(x, y)
        self._translate(x, y)

    def spawn(self, pose_coordinates):
        """
        Spawns the object at given pose_coordinates
        :param pose_coordinates: Position of the object to be spawned
        """
        pass
