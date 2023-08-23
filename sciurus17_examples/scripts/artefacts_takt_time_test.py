#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from std_srvs.srv import Empty, EmptyResponse
from sciurus17_msgs.srv import check_location

from time import sleep
import unittest
import rostest


class taktTimeTest(unittest.TestCase):
    """
    The taktTimeTest Class creates a test that checks the takt time of the movement between picking a block, placing it in a location
    and then returning to the initial position
    """

    def __init__(self, *args):
        super(taktTimeTest, self).__init__(*args)
        rospy.init_node("takt_time_test")

    def test_case(self):
        """
        The test_case function tests the Takt time for the robot to complete a pick and place action, then return to its initial position.
        """

        sleep(60)
        # rospy.wait_for_service("/artefacts/check_location")
        place_check_location_test = rospy.ServiceProxy(
            "/artefacts/check_location", check_location
        )

        # rospy.wait_for_service("/artefacts/pick_and_place")
        sleep(30)
        pick_and_place_test = rospy.ServiceProxy("/artefacts/pick_and_place", Empty)

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
        check_location_response = place_check_location_test.call(block_location.Pose)
        start_time = rospy.Time.now()
        pick_and_place_response = pick_and_place_test.call()
        end_time = rospy.Time.now()
        rospy.logerr(start_time)
        rospy.logerr(end_time)

        takt_time = end_time - start_time

        num_str = str(takt_time)
        double_str = num_str[:2] + "." + num_str[2:4]
        double_value = float(double_str)
        test_message = "takt time is less than 25"

        rospy.logerr(double_value)
        self.assertLess(double_value, 25.00, test_message)


if __name__ == "__main__":
    rostest.rosrun("sciurus17_examples", "takt_time_test", taktTimeTest)
