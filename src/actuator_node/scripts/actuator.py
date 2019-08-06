#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf

from observer_node.msg import Observation, Observations
from math import pi, cos, sin
from robot_control_modules import *

class Actuator:

    def __init__(self, prefix, *args, **kwargs):
        rospy.loginfo("Initializing actuator")
        #self.quat_vertical = tf.transformations.quaternion_from_euler(180*3.1415/180, 0*3.1415/180, 0*3.1415/180, 'rxyz')
        self.prefix = prefix
        #self.tfBuffer = tf2_ros.Buffer()
        #self.listener = tf2_ros.TransformListener(self.tfBuffer, queue_size=1)
        self.init_position()


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
        self.go_to_aligned((0.0, -.3, 0.4))


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
	
        delta_x = -dist * cos(azimuth) * sin(polar)
        delta_y = -dist * sin(azimuth) * sin(polar)
        delta_z = -dist * cos(polar)

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
                
        self.pick_up(obs_4)

    def pick_up(self, observation):

        print("ID: {}".format(observation.id))
        
        #transformed_pose = self.transform(observation.pose, observation.id)
        
        rospy.loginfo("Ready to pick up item at position\n{}".format(observation.pose))

        #secured_position = self.secure_position(transformed_pose)

	#secured_position = (transformed_pose.x, transformed_pose.y, transformed_pose.z)

        secured_position = (observation.pose.pose.position.x, observation.pose.pose.position.y, 
                observation.pose.pose.position.z + 0.12)

        self.go_to_aligned(secured_position)

       # homeRobot(self.prefix)


    def transform(self, pose, obs_id):
        """
        print("------------------!!!TRANSFORM!!!:---------------")

       #This transform represents the transformation from origin (ar_marker_1) to the base of the robot
        transform = self.tfBuffer.lookup_transform("{}link_base".format(self.prefix), pose.header.frame_id, rospy.Time(0))

        #This transform calculates the transformation from the base of the robot arm to the reference ar_marker

        print(type(transform))
        self.print_transform_stamped_pose(transform)
        print("----------------------------------------------------")
        res = tf2_geometry_msgs.do_transform_pose(pose, transform)
        print(type(res))

        return res.pose.position
        """
        pass

    def secure_position(self, position):
        """
        Converts the given position to a safe position.
        This takes into account the boundaries in the real world to avoid collisions with table, wall, ...
        Returns:
        a tuple (x, y, z) with the safe coordinates
        """
        x = max(min(position.x, 0.50), -.50) - 0.02
        if x < -0.10:
            x = -0.14
        if x > 0.08:
            x -= 0.01
        y = min(max(position.y, -1), 0.0) + 0.07
        z = max(min(position.z, 0.33), 0.02) + 0.05
        return (x, y, z)

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
    msg = rospy.wait_for_message('observer_node', Observations)
    act.callback(msg)
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


