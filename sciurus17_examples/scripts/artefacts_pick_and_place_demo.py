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

        self.arm_init()

        while self.model_found == 0:
            print("Waiting for Model coordinates")
        print("Model Coordinates Found")

        self.pick_and_place_service = rospy.Service("/pick_and_place", pick_and_place, self.pick_and_place)
        while self.final_position_found == 0:
            print("Waiting for Place Position to be found")
        print('Place Position Found')
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
        # print(target_place_pose.position.x)


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
        '''
        This subscriber searches through the available models in the world and stores the x, y, and z world coordinte of the desired model for picking up
        '''

        desired_model_name = 'cube3'# This is where you must store the desired model names
        self.item_location = geometry_msgs.msg.Pose()

        model = data.name
        model_index = model.index(desired_model_name) 

        cube_left = data.pose[model_index]
        self.item_location.position.x = cube_left.position.x
        self.item_location.position.y = cube_left.position.y
        self.item_location.position.z = cube_left.position.z

        self.model_found = 1

    def step(self):
        '''
        Step is a function that executes pre-pick, pick, & place motions
        '''
        start_time = rospy.get_rostime()
        print(start_time)
        # self.arm.set_position_target([data.Pose.position.x, data.Pose.position.y, data.Pose.position.z])
        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = self.item_location.position.x
        target_pose.position.y = self.item_location.position.y
        target_pose.position.z = self.item_location.position.z + 0.2
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

# 20208000000

        target_pose.position.x = self.item_location.position.x
        target_pose.position.y = self.item_location.position.y 
        target_pose.position.z = self.item_location.position.z - 0.9 # 0.9 represents the the difference in between where the robot pick the cube and the top of the cube
                                                                 # See if you can tie this offset to any value obtained from world
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

        end_time = rospy.get_rostime()
        print(end_time)

        takt_time =  end_time - start_time
        print(takt_time)

    def arm_init(self):
        '''
        
        '''
        arm_initial_pose = self.arm.get_current_pose().pose

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