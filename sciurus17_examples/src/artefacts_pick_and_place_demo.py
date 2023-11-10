#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
import rosnode
import actionlib
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from gazebo_msgs.msg import ModelStates
from sciurus17_msgs.srv import check_location

from time import sleep


class pickAndPlaceLeft:
    """
    The pick and place class uses the left arm group and left gripper to pick and place objects
    """

    def __init__(self):
        """
        The init function is neccesary to initialize all variables, parameters, and other functions.
        """
        sleep(10)
        self.model_found = 0
        self.final_position_found = 0
        self.robot = moveit_commander.RobotCommander()

        self.arm = moveit_commander.MoveGroupCommander("l_arm_group")
        self.arm.set_max_velocity_scaling_factor(0.1)

        self.gripper = actionlib.SimpleActionClient(
            "/sciurus17/controller2/left_hand_controller/gripper_cmd",
            GripperCommandAction,
        )
        self.gripper.wait_for_server()

        self.gripper_goal = GripperCommandGoal()
        self.gripper_goal.command.max_effort = 2.0

        self.gripper_status = 0

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)

        self.takt_time_pub = rospy.Publisher(
            "/artefacts/takt_time", Float32, queue_size=10
        )

        self.arm_init()

        while self.model_found == 0:
            print("Waiting for Model coordinates")
        print("Model Coordinates Found")

        self.place_check_location_service = rospy.Service(
            "/artefacts/check_location", check_location, self.place_check_location
        )
        while self.final_position_found == 0:
            print("Waiting for Place Position to be found")
        print("Place Position Found")
        self.pick_and_place_service = rospy.Service(
            "/artefacts/pick_and_place", Empty, self.pick_and_place
        )

    def place_check_location(self, data):
        """
        the function receives an x, y, z position for the user service call and determines using the moveit path planner if the arm is able to reach the designated location to place the block
        """
        target_place_pose = geometry_msgs.msg.Pose()
        target_place_pose.position.x = data.Pose.position.x
        target_place_pose.position.y = data.Pose.position.y
        target_place_pose.position.z = data.Pose.position.z
        q = quaternion_from_euler(
            -3.14 / 2.0, 0.0, 0.0
        )  # 上方から掴みに行く場合 When grabbing from above
        target_place_pose.orientation.x = q[0]
        target_place_pose.orientation.y = q[1]
        target_place_pose.orientation.z = q[2]
        target_place_pose.orientation.w = q[3]
        # print(target_place_pose.position.x)

        self.arm.set_pose_target(target_place_pose)  # 目標ポーズ設定

        success, traj_msg, plan_time, err = self.arm.plan()

        if err.val == 1:
            # self.arm.execute(traj_msg)
            self.good_place_pose_x = target_place_pose.position.x
            self.good_place_pose_y = target_place_pose.position.y
            self.good_place_pose_z = target_place_pose.position.z

            self.final_position_found = 1

        return err.val

    def model_state_callback(self, data):
        """
        This subscriber searches through the available models in the world and stores the x, y, and z world coordinte of the desired model for picking up
        """

        desired_model_name = (
            "cube3"  # This is where you must store the desired model names
        )
        self.item_location = geometry_msgs.msg.Pose()

        model = data.name
        model_index = model.index(desired_model_name)

        cube_left = data.pose[model_index]
        self.item_location.position.x = cube_left.position.x
        self.item_location.position.y = cube_left.position.y
        self.item_location.position.z = cube_left.position.z

        self.model_found = 1

    def pick_and_place(self, empty_data):
        """
        Step is a function that executes pre-pick, pick, & place motions
        """

        start_time = rospy.get_rostime()

        print(start_time)

        pre_grip_pose = self.pre_grip()

        self.arm.set_pose_target(pre_grip_pose)  # 目標ポーズ設定

        success, traj_msg, plan_time, err = self.arm.plan()

        if err.val == 1:
            self.arm.execute(traj_msg)

        self.open_gripper()

        grip_pose = self.grip()

        self.arm.set_pose_target(grip_pose)

        self.arm.go()

        self.close_gripper()

        # Hard coded final positon
        # target_pose.position.x = 0.45
        # target_pose.position.y = 0.1
        # target_pose.position.z = 0.13

        place_pose = self.place()

        self.arm.set_pose_target(place_pose)  # 目標ポーズ設定

        success, traj_msg, plan_time, err = self.arm.plan()

        if err.val == 1:
            self.arm.execute(traj_msg)

        self.open_gripper()

        self.arm_init()

        end_time = rospy.get_rostime()
        print(end_time)

        takt_time = end_time - start_time
        print(takt_time)

        takt_time = end_time - start_time

        num_str = str(takt_time)
        double_str = num_str[:2] + "." + num_str[2:4]
        double_value = float(double_str)

        rospy.logerr("publish")
        self.takt_time_pub.publish(double_value)

        return EmptyResponse()

    def arm_init(self):
        """ """
        arm_initial_pose = self.arm.get_current_pose().pose

        self.open_gripper()

        self.arm.set_named_target("l_arm_init_pose")
        self.arm.go()
        self.gripper_goal.command.position = 0.0
        self.gripper.send_goal(self.gripper_goal)
        self.gripper.wait_for_result(rospy.Duration(1.0))

    def pre_grip(self):
        """
        This function is used to get the
        """
        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = self.item_location.position.x
        target_pose.position.y = self.item_location.position.y
        target_pose.position.z = self.item_location.position.z + 0.2

        q = quaternion_from_euler(
            -3.14 / 2.0, 0.0, 0.0
        )  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        return target_pose

    def grip(self):
        """ """
        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = self.item_location.position.x
        target_pose.position.y = self.item_location.position.y
        target_pose.position.z = (
            self.item_location.position.z - 0.9
        )  # 0.9 represents the the difference in between where the robot pick the cube and the top of the cube
        # See if you can tie this offset to any value obtained from world
        q = quaternion_from_euler(
            -3.14 / 2.0, 0.0, 0.0
        )  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        return target_pose

    def place(self):
        """ """
        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = self.good_place_pose_x
        target_pose.position.y = self.good_place_pose_y
        target_pose.position.z = self.good_place_pose_z

        q = quaternion_from_euler(
            -3.14 / 2.0, 0.0, 0.0
        )  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)  # 目標ポーズ設定

        return target_pose

    def move_arm(self, target_location, q, offset):
        """
        Work in progress function
        """
        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = self.target_location.position.x
        target_pose.position.y = self.target_location.position.y
        target_pose.position.z = self.target_location.position.z + (
            offset
        )  # 0.9 represents the the difference in between where the robot pick the cube and the top of the cube
        # See if you can tie this offset to any value obtained from world
        q = quaternion_from_euler(
            -3.14 / 2.0, 0.0, 0.0
        )  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        return target_pose

    def open_gripper(self):
        """
        Function that opens gripper to a fixed width
        """

        # ハンドを開く Open hand
        self.gripper_goal.command.position = -0.7
        self.gripper.send_goal(self.gripper_goal)
        self.gripper.wait_for_result(rospy.Duration(1.0))

        self.gripper_status = 1

        self.arm.go()

    def close_gripper(self):
        """
        Function that closes gripper to a fixed width
        """

        # ハンドを閉じる close the hand
        self.gripper_goal.command.position = -0.4
        self.gripper.send_goal(self.gripper_goal)
        self.gripper.wait_for_result(rospy.Duration(1.0))

        self.gripper_status = 0

        self.arm.go()


if __name__ == "__main__":
    rospy.init_node("artefacts_sciurus17_pick_and_place_controller")

    Mover = pickAndPlaceLeft()

    rospy.spin()

    # rospy.sleep(1.0)
