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
        self.i = 0
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
        while self.i == 0:
            print("Waiting for coordinates")
        self.pick_and_place_service = rospy.Service("/pick_and_place", pick_and_place, self.pick_and_place)

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

        return err.val

    def model_state_callback(self, data):
        if self.i == 1:
            pass
        model = data.name
        # model_index = model[3]
        cube_left = data.pose[3]
        print(cube_left)
        self.item_location = geometry_msgs.msg.Pose()
        self.item_location.position.x = cube_left.position.x
        a = 3.0
        self.item_location.position.y = cube_left.position.y
        self.item_location.position.z = cube_left.position.z
        self.i = 1

    def step(self):
        # self.arm.set_position_target([data.Pose.position.x, data.Pose.position.y, data.Pose.position.z])
        target_pose = geometry_msgs.msg.Pose()
        # data.position.x = 2
        # data.position.y = self.cubeY
        # data.position.z = self.cubeZ

        # # data.position.x = 0.3
        # # data.position.y = 0.1
        # # data.position.z = 0.1
        # q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        # data.orientation.x = q[0]
        # data.orientation.y = q[1]
        # data.orientation.z = q[2]
        # data.orientation.w = q[3]

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

        # print(err)
        # print(success)

        # target_pose.position.x = float('%.3g'%self.cubeX)
        # target_pose.position.x = 0.3
        # target_pose.position.y = 0.1
        # target_pose.position.z = 0.1

        target_pose.position.x = self.item_location.position.x

        target_pose.position.y = self.item_location.position.y 
        target_pose.position.z = self.item_location.position.z - 1
        print(target_pose.position.z)

        # print(self.cubeZ)
        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)  # 目標ポーズ設定
        self.arm.go()

        self.close_gripper()
        self.arm.go()

        # target_pose.position.x = 0.45
        # target_pose.position.y = 0.1
        # target_pose.position.z = 0.13
        # q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        # target_pose.orientation.x = q[0]
        # target_pose.orientation.y = q[1]
        # target_pose.orientation.z = q[2]
        # target_pose.orientation.w = q[3]

        # self.arm.set_pose_target(target_pose)  # 目標ポーズ設定

        # success, traj_msg,  plan_time, err = self.arm.plan()

        # if err.val == 1:
        #     self.arm.execute(traj_msg)

        # self.open_gripper()
        # self.arm.go()

    def arm_init(self):
        '''
        
        '''
        pass


        arm_initial_pose = self.arm.get_current_pose().pose

        print("Arm initial pose:")
        print(arm_initial_pose)


        # # 何かを掴んでいた時のためにハンドを開く Open your hand for when you were holding something

        self.open_gripper()

        # gripper_goal.command.position = -0.9
        # gripper.send_goal(gripper_goal)
        # gripper.wait_for_result(rospy.Duration(1.0))

        # SRDFに定義されている"home"の姿勢にする Set to "home" posture defined in SRDF Set to "home" posture defined in SRDF

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

    def lift(self):
        '''
        
        '''

        # 持ち上げる lift
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.25
        target_pose.position.y = 0.0
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)  # 目標ポーズ設定
        self.arm.go()							# 実行

    def new_position(self):

        '''
        
        '''

        # 移動する Moving
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.4
        target_pose.position.y = 0.0
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)  # 目標ポーズ設定
        self.arm.go()  # 実行

    def place(self):

        '''
        
        '''

        # 下ろす Put down
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.4
        target_pose.position.y = 0.0
        target_pose.position.z = 0.13
        q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)  # 目標ポーズ設定
        self.arm.go()  # 実行

    # ハンドを開く open hand 

        self.open_gripper()
    # gripper_goal.command.position = -0.7
    # gripper.send_goal(gripper_goal)
    # gripper.wait_for_result(rospy.Duration(1.0))

    # 少しだけハンドを持ち上げる lift the hand slightly

    # target_pose = geometry_msgs.msg.Pose()
    # target_pose.position.x = 0.4
    # target_pose.position.y = 0.0
    # target_pose.position.z = 0.2
    # q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)  # 上方から掴みに行く場合 When grabbing from above
    # target_pose.orientation.x = q[0]
    # target_pose.orientation.y = q[1]
    # target_pose.orientation.z = q[2]
    # target_pose.orientation.w = q[3]

    # arm.set_pose_target(target_pose)  # 目標ポーズ設定
    # arm.go()  # 実行

# # box_position_x = 0.3
# # box_position_y = -0.1
# # box_position_z = 1.01


    # SRDFに定義されている"home"の姿勢にする
        self.arm.set_named_target("l_arm_init_pose")
        self.arm.go()

    print("done")




if __name__ == '__main__':

    rospy.init_node("sciurus17_pick_and_place_controller")

    Mover = pick_and_place_left()

    rospy.spin()

    # rospy.sleep(1.0)