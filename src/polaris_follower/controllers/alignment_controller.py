#!/usr/bin/env
from time import sleep

import rospy
import message_filters

from polaris_follower.constants import *
from polaris_follower.controllers.base_controller import BaseController


class AlignmentController(BaseController):
    """
    Given a source and a destination object, this controller aligns the source object in the direction
    of the destination object.
    """
    def __init__(self, src, dest):
        """
        @param src: Source node
        @param dest: Destination node
        """
        super().__init__()
        self.src = src
        self.dest = dest

        src_vel_tpc = self.src.get_vel_topic()
        self.src_vel_pub = rospy.Publisher(src_vel_tpc[KEY_TOPIC_NAME],
                                           src_vel_tpc[KEY_TOPIC_MSG_TYPE], queue_size=10)

    def simulate(self):
        """
        Simulating the robots
        """
        sleep(1)    # Sleep so that the robot positions can be updated first
        self.src.align_with_dest(self.dest)
        rospy.spin()
