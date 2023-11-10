#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from std_msgs.msg import Float32
from std_msgs.msg import Float32

from std_srvs.srv import Empty, EmptyResponse
from sciurus17_msgs.srv import check_location

from time import sleep
import unittest
import rostest


class TestTaktTime(unittest.TestCase):
    """
    The taktTimeTest Class creates a test that checks the takt time of the movement between picking a block, placing it in a location
    and then returning to the initial position
    """

    def __init__(self, *args):
        super(TestTaktTime, self).__init__(*args)
        rospy.init_node("takt_time_test")
        rospy.Subscriber("/artefacts/takt_time", Float32, self.takt_time_callback)
        self.place_check_location_test = rospy.ServiceProxy(
            "/artefacts/check_location", check_location
        )
        self.pick_and_place_test = rospy.ServiceProxy(
            "/artefacts/pick_and_place", Empty
        )

    def takt_time_callback(self, data):
        """
        Callback that gets triggered when a message is published by the takt time publisher.
        If a message is published then takt_time_ok is true allowing the test to pass
        """

        rospy.loginfo(data)
        self.takt_time_ok = True

    def test_case(self):
        """
        The test_case function tests the Takt time for the robot to complete a pick and place action, then return to its initial position.
        """

        rospy.wait_for_service("/artefacts/check_location")

        block_location = check_location()

        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = 0.45
        target_pose.position.y = 0.1
        target_pose.position.z = 0.13

        target_pose.orientation.x = 0
        target_pose.orientation.y = 0
        target_pose.orientation.z = 0
        target_pose.orientation.w = 1

        block_location.Pose = target_pose
        check_location_response = self.place_check_location_test.call(target_pose)

        sleep(1)

        pick_and_place_response = self.pick_and_place_test.call()

        sleep(1)

        self.assertTrue(self.takt_time_ok)


if __name__ == "__main__":
    rostest.rosrun("sciurus17_examples", "test_takt_time", TestTaktTime)
