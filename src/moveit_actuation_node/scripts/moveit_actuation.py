#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonInterfaceTutorial:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        self.robot = moveit_commander.RobotCommander() # This is the outer level interface to the robot
        self.scene = moveit_commander.PlanningSceneInterface() # This is an interface to the worl surrouding the robot

        # MoveGroupCommander is an interface to a group of joints beloning to the robot as defined with the moveit setup assistent
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
    
        #The DisplayTrajectory publisher can be used to visualize planned trajectories in rviz!!
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        #Getting some basic information:
        self.planning_frame_arm = self.arm_group.get_planning_frame()
        print("=================== Reference frame for arm group: %s"%self.planning_frame_arm)

        self.planning_frame_gripper = self.gripper_group.get_planning_frame()
        print("=================== Reference frame for the gripper group: %s"%self.planning_frame_gripper)

        print("=================== Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def go_to_pose_goal(self):

        self.arm_group.set_planning_time(10)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.00678814403766
        pose_goal.orientation.y = 0.0132801833852
        pose_goal.orientation.z = -0.0130724789272
        pose_goal.orientation.w = 0.99980331472

        pose_goal.position.x = 0.257340085722
        pose_goal.position.y = -0.410991776969
        pose_goal.position.z = -0.0773751339307

        self.arm_group.set_pose_target(pose_goal)

        #calling the planner to compute a plan
        plan = self.arm_group.go(wait=True)

        #Calling stop ensures that there is no residual movement
        self.arm_group.stop()

        #It is always a good idea to clear targets after planning with poses
        self.arm_group.clear_pose_targets()

        #self.execute_plan(plan)

    def execute_plan(self, plan):
        try:
            self.arm_group.execute(plan, wait=True)
        except rospy.ROSInterruptException:
            return

def main():
    move_group = MoveGroupPythonInterfaceTutorial()

    move_group.go_to_pose_goal()


if __name__ == '__main__':
    main()

