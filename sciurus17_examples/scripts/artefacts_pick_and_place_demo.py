#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
# import gazebo_msgs.msg

import rosnode
import actionlib
from tf.transformations import quaternion_from_euler
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from gazebo_msgs.msg import ModelStates
from sciurus17_msgs.srv import pick_and_place

from time import sleep

class pick_and_place_left():
    '''
    The pick and place class 
    '''

    def __init__(self):
        '''
        The init function is neccesary to initialize all variables, parameters, and other functions. 
        '''
        self.model_found = 0
        self.final_position_found = 0
        self.robot = moveit_commander.RobotCommander()

        self.arm = moveit_commander.MoveGroupCommander("l_arm_group")
        self.arm.set_max_velocity_scaling_factor(0.1)

        self.gripper = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)
        self.gripper.wait_for_server()

        self.gripper_goal = GripperCommandGoal()
        self. gripper_goal.command.max_effort = 2.0

        self.gripper_status = 0

        ############################
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)
        # print(self.model_coordinates)
        # self.object_coordinates = self.model_coordinates("8115RC_V2", "")
        self.arm_init()
        while self.model_found == 0:
            print("Waiting for Model coordinates")
        print("Model Coordinates Found")
        self.pick_and_place_service = rospy.Service("/pick_and_place", pick_and_place, self.pick_and_place)
        while self.final_position_found == 0:
            print("Waiting for Place Position to be found")
        self.step()

    def pick_and_place(self, data):
        target_place_pose = geometry_msgs.msg.Pose()
        target_place_pose.position.x = data.Pose.position.x
        target_place_pose.position.y = data.Pose.position.y
        target_place_pose.position.z = data.Pose.position.z
        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_place_pose.orientation.x = q[0]
        target_place_pose.orientation.y = q[1]
        target_place_pose.orientation.z = q[2]
        target_place_pose.orientation.w = q[3]
        print(target_place_pose.position.x)


        self.arm.set_pose_target(target_place_pose)  # 目標ポーズ設定

        success, traj_msg,  plan_time, err = self.arm.plan()

        if err.val == 1:
            # self.arm.execute(traj_msg)
            self.good_place_pose_x = target_place_pose.position.x
            self.good_place_pose_y = target_place_pose.position.y
            self.good_place_pose_z = target_place_pose.position.z

            self.final_position_found = 1

        return err.val

    def model_state_callback(self, data):
        # if self.model_found == 1:
        #     pass
        model = data.name
        # model_index = model[3]
        cube_left = data.pose[3]
        self.item_location = geometry_msgs.msg.Pose()
        self.item_location.position.x = cube_left.position.x
        a = 3.0
        self.item_location.position.y = cube_left.position.y
        self.item_location.position.z = cube_left.position.z
        self.model_found = 1

    def step(self):
        '''
        Step is a function that executes pre-pick, pick, & place motions
        '''
        # self.arm.set_position_target([data.Pose.position.x, data.Pose.position.y, data.Pose.position.z])
        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = 0.2511
        target_pose.position.y = 0.0
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)  # 目標ポーズ設定

        success, traj_msg,  plan_time, err = self.arm.plan()

        if err.val == 1:
            self.arm.execute(traj_msg)

        self.open_gripper()
        self.arm.go()							# 実行


        target_pose.position.x = self.item_location.position.x

        target_pose.position.y = self.item_location.position.y 
        target_pose.position.z = self.item_location.position.z - 0.9 # 0.9 represents the the difference in between where the robot pick the cube and the top of the cube
        print(target_pose.position.z)

        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)
        self.arm.go()

        self.close_gripper()
        self.arm.go()

        # Hard coded final positon
        # target_pose.position.x = 0.45
        # target_pose.position.y = 0.1
        # target_pose.position.z = 0.13
        target_pose.position.x = self.good_place_pose_x
        target_pose.position.y = self.good_place_pose_y
        target_pose.position.z = self.good_place_pose_z

        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)  # 目標ポーズ設定

        success, traj_msg,  plan_time, err = self.arm.plan()

        if err.val == 1:
            self.arm.execute(traj_msg)

        self.open_gripper()
        self.arm.go()

        self.arm_init()


    def arm_init(self):
        '''
        
        '''
        pass


        arm_initial_pose = self.arm.get_current_pose().pose

        print("Arm initial pose:")
        print(arm_initial_pose)


        # # 何かを掴んでいた時のためにハンドを開く Open your hand for when you were holding something

        self.open_gripper()

        self.arm.set_named_target("l_arm_init_pose")
        self.arm.go()
        self.gripper_goal.command.position = 0.0
        self.gripper.send_goal(self.gripper_goal)
        self.gripper.wait_for_result(rospy.Duration(1.0))

    def open_gripper(self):

        '''
        
        '''

        # ハンドを開く Open hand
        self.gripper_goal.command.position = -0.7
        self.gripper.send_goal(self.gripper_goal)
        self.gripper.wait_for_result(rospy.Duration(1.0))

        self.gripper_status = 1

    def close_gripper(self):

        '''
        
        '''

        # ハンドを閉じる close the hand
        self.gripper_goal.command.position = -0.4
        self.gripper.send_goal(self.gripper_goal)
        self.gripper.wait_for_result(rospy.Duration(1.0))

        self.gripper_status = 0
						# 実行



if __name__ == '__main__':

    rospy.init_node("sciurus17_pick_and_place_controller")

    Mover = pick_and_place_left()

    rospy.spin()

    # rospy.sleep(1.0)