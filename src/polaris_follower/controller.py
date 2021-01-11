#!/usr/bin/env

import rospy
import message_filters

from polaris_follower.constants import *


class Controller:
    def __init__(self, src, dest):
        """
        @param src: Source node
        @param dest: Destination node
        """
        self.src = src
        self.dest = dest

        src_vel_tpc = self.src.get_vel_topic()
        self.src_vel_pub = rospy.Publisher(src_vel_tpc[KEY_TOPIC_NAME],
                                           src_vel_tpc[KEY_TOPIC_MSG_TYPE], queue_size=10)

    def move_cb(self, src_pose, dest_pose):
        """
        Callback function to align a source object in the direction of dest object
        @param src_pose: Pose of Src object to align
        @param dest_pose: Pose of Dest object to get direction of alignment
        """
        # Create the twist message to publish
        self.src.set_pose(src_pose)
        self.dest.set_pose(dest_pose)
        msg = self.src.create_alignment_msg(self.dest.get_position_coordinates())

        # Publish the message
        self.src_vel_pub.publish(msg)

    def simulate(self):
        src_pose_tpc = self.src.get_pose_topic()
        dest_pose_tpc = self.dest.get_pose_topic()

        # Subscribe to the pose nodes for turtle1 and turtle2
        src_pose_sub = message_filters.Subscriber(src_pose_tpc[KEY_TOPIC_NAME],
                                                  src_pose_tpc[KEY_TOPIC_MSG_TYPE])
        dest_pose_sub = message_filters.Subscriber(dest_pose_tpc[KEY_TOPIC_NAME],
                                                   dest_pose_tpc[KEY_TOPIC_MSG_TYPE])

        # Time synchronizer to get the topics all at once
        ts = message_filters.ApproximateTimeSynchronizer([src_pose_sub, dest_pose_sub], 10, slop=0.1,
                                                         allow_headerless=True)
        # Register callback function
        ts.registerCallback(self.move_cb)
        rospy.spin()
