from polaris_follower.constants import *


class Base:
    def __init__(self, turtle_name, pose_topic_substr, vel_topic_substr, pose_msg_type,
                 vel_msg_type, namespace='turtlesim_align'):
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

        # Spawn service
        self.spawn_service = str.format('/{}/spawn', self.object_ns)

        self.pose = pose_msg_type()

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
        pass

    def create_alignment_msg(self, dest_pos_coordinates):
        """
        Given position coordinates (x,y, theta), it creates a message
        to be passed on to the velocity pubisher
        :param dest_pos_coordinates: (x, y, theta)
        :return: Vel message
        """
        pass

    def spawn(self, pose_coordinates):
        """
        Spawns the object at given pose_coordinates
        :param pose_coordinates: Position of the object to be spawned
        """
        pass
