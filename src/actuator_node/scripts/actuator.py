#!/usr/bin/env python

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs
import geometry_msgs
import tf2_ros
import tf2_geometry_msgs
import tf

import actionlib
import kinova_msgs.msg
import math

from moveit_commander.conversions import pose_to_list
from observer_node.msg import Observation, Observations
from robot_control_modules import *
from std_msgs.msg import String

BLUE_ID = 4
RED_ID = 6
YELLOW_ID = 7

class Actuator:

    def __init__(self, prefix, *args, **kwargs):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander() # This is the outer level interface to the robot
        self.scene = moveit_commander.PlanningSceneInterface() # This is an interface to the worl surrouding the robot

        # MoveGroupCommander is an interface to a group of joints beloning to the robot as defined with the moveit setup assistent
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
    
        #The DisplayTrajectory publisher can be used to visualize planned trajectories in rviz!!
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        rospy.loginfo("Initializing actuator")
        #self.quat_vertical = tf.transformations.quaternion_from_euler(180*3.1415/180, 0*3.1415/180, 0*3.1415/180, 'rxyz')
        self.prefix = prefix
        #self.tfBuffer = tf2_ros.Buffer()
        #self.listener = tf2_ros.TransformListener(self.tfBuffer, queue_size=1)
        self.init_position()

        self.current_cartesian_command = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home position in mq format (meters - quaternion)

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


    def init_position(self):
        rospy.loginfo('Going home')
	homeRobot(self.prefix)
        print("Robot Arm is now in home position")       
        self.go_home()
        self.open_gripper()

    def go_home(self):

	"""
	Uses the standard home values from the factory settings of the robot:
	position = (x, y, z) == (0.2108822746158, -0.25810956955, 0.509347975254)
	orientation = (qx ,qy, qz, qw) == (0.644038379192, 0.319248020649, 0.420018672943, 0.553967058659)

	"""
        #position = (0.2108822746158, -0.25810956955, 0.509347975254)
        #quat_orientation = (0.644038379192, 0.319248020649, 0.420018672943, 0.553967058659)
        #cartesian_pose_client(position, quat_orientation, self.prefix)

        #self.go_to_aligned((0.0, -.3, 0.4))
        
        angles = [275.238983154, 231, 153, 314, 126, 137]
        print(angles)
        plan = self.go_to_joint_goal(list(map(lambda x : math.radians(x), angles)))
        print("PLAN")
        print(plan)

        self.execute_plan(plan)


    def test_marker(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.2721
        pose_goal.position.y = -0.4239
        pose_goal.position.z = 0.02
    
        #self.arm_group.set_pose_target(pose_goal)
        #plan = self.arm_group.plan(pose_goal)

        plan, fraction = self.go_to_cartesian(pose_goal)

        print("PLAN")
        print(plan)
        self.execute_plan(plan)

    def go_to_aligned(self, position):
	"""
	position is a tuple (x, y, z) where:
	--> the positive x-axis is directed to the left when standing behind the robo arm (i.e. when facing the base plate containing the connections)
	--> the positive y-axis is directed towards the user when standing behind the robot (i.e facing the base plate) this mean that negative y-values will make the robot gripper move forward.
	--> the positive z-axis is directed when facing the base plate
	"""
        p, q = self.generate_gripper_align_pose(position, 0.0, math.pi/2.0, 0.0, 0.0)

	#p, q = self.generate_gripper_align_pose(position, 0.03, math.pi/2.0, 0.0, 0.0)

        return cartesian_pose_client(p, q, self.prefix)
        

    def generate_gripper_align_pose(self, target_position, dist, azimuth, polar, rot_gripper_z):
	
	"""
	Generate an aligned pose at the given target position.
	Code adapted from C++ implementation at https://github.com/Kinovarobotics/kinova-ros/blob/master/kinova_moveit/kinova_arm_moveit_demo/src/pick_place.cpp
	Arguments:
	target_position -- tuple (x, y, z) identifying the target position
	dist -- distance of returned position to target_position
	azimuth -- an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi)
	polar -- also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
	rot_gripper_z -- rotation along the z axis of the gripper reference frame (last joint rotation)
	Returns:
	a tuple ((px, py, pz), q) where (px, py, pz) is the target position and q is the target quaternion for the robotarm.
   	"""
	
        delta_x = -dist * math.cos(azimuth) * math.sin(polar)
        delta_y = -dist * math.sin(azimuth) * math.sin(polar)
        delta_z = -dist * math.cos(polar)

        q = tf.transformations.quaternion_from_euler(
            azimuth, polar, rot_gripper_z, 'rxyz')

        px = target_position[0] + delta_x
        py = target_position[1] + delta_y
        pz = target_position[2] + delta_z

        return ((px, py, pz), q)

    def open_gripper(self):
        """
        Opens the gripper of the robotarm.
        """
        rospy.loginfo('Opening gripper')
        return gripper_client([0, 0, 0], self.prefix)

    def close_gripper(self):
        """
        Closes the gripper of the robotarm.
        """
        rospy.loginfo('Closing gripper')
        return gripper_client([6400, 6400, 0], self.prefix)

    def callback_main_node(self, msg):
        print(msg.data)

        action = msg.data.split("(", 1)

        action_1 = action[0]
        action_2 = action[1].split("(", 1)
    
        blocks = action_2[1].split(",")
        
        print("The blocks in this operations are: ")
        print(blocks)

        operator_1_id = self.get_block_id(blocks[0].split("(",1)[0].strip())
        operator_2_id = self.get_block_id(blocks[1].split("(",1)[0].strip())

        print("Waiting for an observation from the kinect camera")
        observations_msg = rospy.wait_for_message('observer_node', Observations)
        
        print(observations_msg)

        block_1 = None
        block_2 = None

        for observation in observations_msg.observations:
            if observation.id == operator_1_id:
                block_1 = observation
            if observation.id == operator_2_id:
                block_2 = observation

        print("BLOCK 1")
        print(block_1)

        print("BLOCK 2")
        print(block_2)

        print("Ready to start pick-up operation")
        self.pick_up(block_1)

        print("Ready to put down the block")
        self.put_down(block_2)
    
    def get_block_id(self, block):

        if block.lower() == "blue":
            return BLUE_ID
        elif block.lower() == "red":
            return RED_ID
        elif block.lower() == "yellow":
            return YELLOW_ID

    def callback(self, msg):

        obs_4 = None
        obs_6 = None

        for obs in msg.observations:
            print("--------")
            print("ar_marker_{}".format(obs.id))
            point = obs.pose.pose.position
            orientation = obs.pose.pose.orientation
            print("The position of the marker is x:{} y:{} z:{}".format(point.x, point.y, point.z))
            print("The orientation of the marker is qx:{} qy:{} qz:{} qw:{}".format(orientation.x, orientation.y, orientation.z, orientation.w))
            if int(obs.id) == 4:
                obs_4 = obs
            if int(obs.id) == 6:
                obs_6 = obs


        self.pick_up(obs_4)
        self.put_down(obs_6)        

    def pick_up(self, observation):


        pre_pickup_pose = self.generate_pre_pickup_pose(observation.pose.pose)
        
        print("GOING TO SAFE POSE FOR PICKUP")
        plan_pre_pickup, fraction = self.go_to_cartesian(pre_pickup_pose)
        print("PLAN")
        print(plan_pre_pickup)
        print("==========================")
        print("FRACTION")
        print(fraction)
        print("==========================")
        print("Ready to execute plan")
        self.execute_plan(plan_pre_pickup)

        print("GOING TO PICKUP POSE")
        pickup_pose = self.generate_pickup_pose(pre_pickup_pose)
        plan_pickup, fraction = self.go_to_cartesian(pickup_pose)
        self.execute_plan(plan_pickup)

        print("GRABBING OBJECT")
        gripper_client([3000.0, 3000.0, 0], self.prefix)

        print("BRINGING OBJECT HOME")
        self.go_home()

    def generate_pre_pickup_pose(self, unsafe_pose):

        safe_pose = geometry_msgs.msg.Pose()
        safe_pose.position.x = unsafe_pose.position.x
        safe_pose.position.y = unsafe_pose.position.y
        safe_pose.position.z = unsafe_pose.position.z + 0.10

        return safe_pose

    def generate_pickup_pose(self, pre_pickup_pose):
        
        pickup_pose = geometry_msgs.msg.Pose()
        pickup_pose.position.x = pre_pickup_pose.position.x
        pickup_pose.position.y = pre_pickup_pose.position.y
        pickup_pose.position.z = pre_pickup_pose.position.z - 0.10

        return pickup_pose
    
    def put_down(self, observation):
        
        pre_drop_pose = self.generate_pre_drop_pose(observation.pose.pose)

        print("GOING TO SAFE POSE FOR DROP")
        plan_pre_drop, fraction = self.go_to_cartesian(pre_drop_pose)
        print("PLAN")
        print(plan_pre_drop)
        print("==========================")
        print("FRACTION")
        print(fraction)
        print("==========================")
        print("Ready to execute plan")
        self.execute_plan(plan_pre_drop)

        print("GOING TO DROP POSE")
        drop_pose = self.generate_drop_pose(pre_drop_pose)
        plan_drop, fraction = self.go_to_cartesian(drop_pose)
        self.execute_plan(plan_drop)

        print("DROPPING OBJECT")
        self.open_gripper()

        print("GOING TO SAFE POSE TO CONTINUE POST DROP")
        post_drop_pose = self.generate_post_drop_pose(drop_pose)
        plan_post_drop, _ = self.go_to_cartesian(post_drop_pose)
        self.execute_plan(plan_post_drop)

        print("GOING HOME")
        self.go_home()

    def generate_pre_drop_pose(self, unsafe_pose):
        
        safe_pose = geometry_msgs.msg.Pose()
        safe_pose.position.x = unsafe_pose.position.x
        safe_pose.position.y = unsafe_pose.position.y
        safe_pose.position.z = unsafe_pose.position.z + 0.10

        return safe_pose

    def generate_drop_pose(self, pre_drop_pose):
        
        drop_pose = geometry_msgs.msg.Pose()
        drop_pose.position.x = pre_drop_pose.position.x
        drop_pose.position.y = pre_drop_pose.position.y
        drop_pose.position.z = pre_drop_pose.position.z - 0.03

        return drop_pose

    def generate_post_drop_pose(self, drop_pose):
        
        post_drop_pose = geometry_msgs.msg.Pose()
        post_drop_pose.position.x = drop_pose.position.x
        post_drop_pose.position.y = drop_pose.position.y
        post_drop_pose.position.z = drop_pose.position.z + 0.20

        return post_drop_pose

    def go_to_cartesian(self, desired_pose):
        
        waypoints=[]

        safe_position = self.arm_group.get_current_pose().pose
        safe_position.position.x = desired_pose.position.x
        safe_position.position.y = desired_pose.position.y
        safe_position.position.z = desired_pose.position.z
        waypoints.append(copy.deepcopy(safe_position))

        safe_position.position.z = desired_pose.position.z   
        waypoints.append(copy.deepcopy(safe_position))

        (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,
                0.01,
                0.0)
        
        return plan, fraction

    def go_to_joint_goal(self, joint_configuration):
        joint_goal = self.arm_group.get_current_joint_values()
        
        print(joint_goal)

        joint_goal[0] = joint_configuration[0]
        joint_goal[1] = joint_configuration[1]
        joint_goal[2] = joint_configuration[2]
        joint_goal[3] = joint_configuration[3]
        joint_goal[4] = joint_configuration[4]
        joint_goal[5] = joint_configuration[5]

        plan = self.arm_group.plan(joint_goal)

        return plan

    def execute_plan(self, plan):
        try:
            self.arm_group.execute(plan, wait=True)
        except rospy.ROSInterruptException:
            return
    
    def push_relative_cartesian_pose(self, position_list, orientation_list):
        action_address = '/' + self.prefix + 'driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=(self.prefix+'link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(x=position_list[0], y=position_list[1], 
                z=position_list[2] + 0.10)
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
                x=orientation_list[0], y=orientation_list[1], z=orientation_list[2], w=orientation_list[3])

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('         the cartesian action timed-out')
            return None


    def generate_relative_pose(self, position_tuple, orientation_tuple):
        self.get_current_cartesian_command()
        
        position_list = list(position_tuple)
        relative_position_list = position_list

        for i in range(0,3):
            relative_position_list[i] = position_list[i] + self.current_cartesian_command[i]

        orientation_XYZ = self.quaternion_2_euler_xyz(orientation_tuple)
        orientation_XYZ_list = [orientation_XYZ[i] + self.current_cartesian_command[3+i] for i in range(0,3)]
        relative_orientation_list = self.euler_xyz_2_quaternion(orientation_XYZ_list)
        
        return relative_position_list + relative_orientation_list


    def quaternion_2_euler_xyz(self, quaternion_tuple):
        qx, qy, qz, qw = self.quaternion_norm(quaternion_tuple)[0:4]

        euler_x = math.atan2((2 * qw * qx - 2 * qy * qz), (qw * qw - qx * qx - qy * qy + qz * qz))
        euler_y = math.asin(2 * qw * qy + 2 * qx * qz)
        euler_z = math.atan2((2 * qw * qz - 2 * qx * qy), (qw * qw + qx * qx - qy * qy - qz * qz))

        return [euler_x, euler_y, euler_z]


    def quaternion_norm(self, quaternion_tuple):
        qx_temp, qy_temp, qz_temp, qw_temp = quaternion_tuple[0:4]
        qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
        qx = qx_temp/qnorm
        qy = qy_temp/qnorm
        qz = qz_temp/qnorm       
        qw = qw_temp/qnorm

        return [qx, qy, qz, qw] # the normed quaternion

    def euler_xyz_2_quaternion(self, euler_xyz_list):
        euler_x, euler_y, euler_z = euler_xyz_list[0:3]

        sx = math.sin(0.5 * euler_x)
        cx = math.cos(0.5 * euler_x)
        sy = math.sin(0.5 * euler_y)
        cy = math.cos(0.5 * euler_y)
        sz = math.sin(0.5 * euler_z)
        cz = math.cos(0.5 * euler_z)

        qx = sx * cy * cz + cx * sy * sz
        qy = -sx * cy * sz + cx * sy * cz
        qz = sx * sy * cz + cx * cy * sz
        qw = -sx * sy * sz + cx * cy * cz

        return [qx, qy, qz, qw]

    def get_current_cartesian_command(self):
        topic_address = '/' + self.prefix + 'driver/out/cartesian_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.set_current_cartesian_command)
        rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
        print("Cartesian command listener obtained message for current cartesian pose")

    def set_current_cartesian_command(self, msg):
        """
        The format of the messags on topic /m1n6s200_driver/out/cartesian_command is:
            X: float64
            Y: float64
            Z: float64
            ThetaX: float64
            ThetaY: float64
            ThetaZ: float64
        """
        current_cartesian_command_str_list = str(msg).split("\n")

        for index in range(0, len(current_cartesian_command_str_list)):
            temp_str = current_cartesian_command_str_list[index].split(": ")
            self.current_cartesian_command[index] = float(temp_str[1])

    
    def print_transform_stamped_pose(self, transform_stamped):
        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation

        print("POSITION: x:{}, y={}, z:{}".format(translation.x, translation.y, translation.z))
        print("ORIENTATION: qx:{}, qy:{}, qz:{}, gw:{}".format(rotation.x, rotation.y, rotation.z, rotation.w))


    

def actuator_main():
    rospy.init_node('actuator_node')
    act = Actuator('m1n6s200_')

    print("Actuator has been initialized")

    #rospy.Subscriber("observer_node", Observations, act.callback, queue_size=1)
   # rospy.Service('actuator_service' Action, act.handle_action)

    msg = rospy.wait_for_message('central_to_actuator', String)
    act.callback_main_node(msg)
    rospy.loginfo("Ready to spin")
    rospy.spin()

if __name__ == "__main__":
    try:
       # user = expanduser("~")
       # path = user + "/DTAI_Internship/src/actuator_node/scripts/"
       # subprocess.call(['sh', path + "kinova.sh"])
        actuator_main()
    except rospy.ROSInterruptException:
        pass


