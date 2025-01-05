#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from detach import detach_links

class MoveItControlDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            DisplayTrajectory,
                                                            queue_size=20)

        self.group_name_arm = "arm"
        self.group_name_gripper = "gripper"
        self.move_group_arm = MoveGroupCommander(self.group_name_arm)
        self.move_group_gripper = MoveGroupCommander(self.group_name_gripper)
        
        self.reference_frame = 'base_link'
        self.move_group_arm.set_pose_reference_frame(self.reference_frame)
        self.move_group_arm.allow_replanning(True)
        self.move_group_arm.set_goal_position_tolerance(0.05)
        self.move_group_arm.set_goal_orientation_tolerance(0.05)
        
        self.end_effector_link = self.move_group_arm.get_end_effector_link()

    def go_to_pose(self, pose):
        self.move_group_arm.set_start_state_to_current_state()
        self.move_group_arm.set_pose_target(pose, self.end_effector_link)
        plan = self.move_group_arm.go(wait=True)
        rospy.sleep(1)
        return plan

    def detach_objects(self, model_name_1, link_name_1, model_name_2, link_name_2):
        return detach_links(model_name_1, link_name_1, model_name_2, link_name_2)

    def execute_named_target(self, target_name):
        if target_name in ["open", "catch"]:
            self.move_group_gripper.set_named_target(target_name)
            self.move_group_gripper.go(wait=True)
        else:
            self.move_group_arm.set_named_target(target_name)
            self.move_group_arm.go(wait=True)
        rospy.sleep(1)

    def main(self):
        
        self.execute_named_target("place")
        
        
        self.execute_named_target("open")
        
        self.detach_objects("unit_cylinder_0", "link", "robot", "right_gripper")
        
        self.execute_named_target("upup")
        
    

        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    moveit_demo = MoveItControlDemo()
    moveit_demo.main()
