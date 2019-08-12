#!/usr/bin/env python

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from observer_node.msg import Observation, Observations

class MoveGroupPythonInterfaceTutorial:

    def __init__(self):
        #joint_state_topic = ['joint_states:=/m1n6s200_driver/out/joint_state']
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


        print("=================== End effector link for arm group")
        self.end_effector = self.arm_group.get_end_effector_link()
        print(self.end_effector)

        print("=================== Current pose of the end effector for the arm group")
        print(self.arm_group.get_current_pose(self.end_effector))

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

    def go_to_cartesian(self, desired_pose):
        
        waypoints=[]

        current_pose = self.arm_group.get_current_pose().pose
        current_pose.position.x += desired_pose.position.x 
        current_pose.position.y += desired_pose.position.y
        current_pose.position.z += desired_pose.position.z
        waypoints.append(copy.deepcopy(current_pose))
    
        #current_pose.position.z += desired_pose.position.z   
        #waypoints.append(copy.deepcopy(current_pose))

        (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,
                0.01,
                0.0)
        
        return plan, fraction
    

    def execute_plan(self, plan):
        try:
            self.arm_group.execute(plan, wait=True)
        except rospy.ROSInterruptException:
            return

    def callback(self, msg):

        obs_4 = None

        for obs in msg.observations:
            print("--------")
            print("ar_marker_{}".format(obs.id))
            point = obs.pose.pose.position
            orientation = obs.pose.pose.orientation
            print("The position of the marker is x:{} y:{} z:{}".format(point.x, point.y, point.z))
            print("The orientation of the marker is qx:{} qy:{} qz:{} qw:{}".format(orientation.x, orientation.y, orientation.z, orientation.w))
            if int(obs.id) == 4:
                obs_4 = obs

        plan, fraction = self.go_to_cartesian(obs.pose.pose)

        print("PLAN")
        print(plan)
        print("==========================")
        print("FRACTION")
        print(fraction)
        print("==========================")
        print("Ready to execute plan")
        self.execute_plan(plan)

        return plan, fraction
                
def main():
    move_group = MoveGroupPythonInterfaceTutorial()

    msg = rospy.wait_for_message('observer_node', Observations)

    move_group.callback(msg)

if __name__ == '__main__':
    main()

