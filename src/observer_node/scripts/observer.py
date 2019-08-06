#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import tf2_geometry_msgs
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from observer_node.msg import Observation, Observations


REFERENCE_TAG_ID = 2
ROBOT_TAG_ID = 1
ORIGIN_FRAME = "origin"
ROBOT_BASE_FRAME = "m1n6s200_link_base"

class Observer:

    def __init__(self, **args):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.origin_marker_frame = "ar_marker_{}".format(ROBOT_TAG_ID)
        self.camera_frame = "kinect2_link"

        self.publisher = rospy.Publisher('observer_node', Observations, queue_size=1)
        rospy.loginfo("Initialized {}".format(self))
    
    def callback(self, msg):
        """
        This method serves as the callback for the ar_pose_marker topic to which the observer
        is subscribed. This callback method will tranform the detected ar tags and publish them
        on the /observer_node topic.
        The transformations that occur will have as main purpose to represent coordinates on
        the plane relative to the robot base.
        This means that there are at least 2 transforms:
            -   The static transform representing the transformation from ar_marker_1 to the base
                the robot --> this transform is included in the launch file following the
                guidelines on intended use of static transforms.
            -   The transform representing the transformation from the kinect2_link frame to
                ar_marker_1 which will be seen as the origin. --> this transform is defined here
                because it is dependent on the distance (and location) between the kinect camera
                and the robot arm.
                --> This transform is already being broadcasted by the ar_track_alvar node which is 
                defined in the observer_node.launch file. The target frame of this broadcast
                is redirected to "kinect2_link" so this means that to make ar_marker_1 the origin,
                the lookup_transform method can be called on the buffer:
                    transform = self.tfBuffer.lookup_transform("ar_marker_1", "kinect2_link",..)
                Then, the coordinates of another ar_tag, as measured from the kinect2, can be
                transformed with respect to the newly defined origin ar_marker_1 by calling:
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(marker.pose, transform)
                        --> this transformed_pose then contains the coordinates of the new 
                            ar_marker with respect to the origin marker.
                Then, to get the coordinate of the new marker with respect to the robot base, 
                the static transform can be retrieved by subsequently calling:
                    static_transform = self.tfBuffer("m1n6s200_link_base", "ar_marker_1",...)
                    target_pose = tf2_geometry_msgs.do_transform_pose(marker.pose, static_transform)
        """
        rospy.loginfo("Transforming markers {}".format(self))
        new_msg = Observations()
        new_msg.header = msg.header

        # The following transform will make the origin_marker (ar_marker_1) the origin of the frame
        self.transform = self.tf_buffer.lookup_transform(self.origin_marker_frame, self.camera_frame, rospy.Time(0), rospy.Duration(5))

        self.static_transform = self.tf_buffer.lookup_transform(ROBOT_BASE_FRAME, self.origin_marker_frame, rospy.Time(0))

        for marker in msg.markers:
            if marker.id in [ROBOT_TAG_ID]:
                continue #The robot tag is not needed right now
            try:
                rospy.loginfo("marker {} measured_pose {}".format(
                    marker.id, self.prettify(marker.pose.pose.position)))

                transformed_pose = tf2_geometry_msgs.do_transform_pose(
                        marker.pose, self.transform)
               
                rospy.loginfo("marker {} transformed_pose with respect to origin is {}".format(
                    marker.id, self.prettify(transformed_pose.pose.position)))

                target_pose = tf2_geometry_msgs.do_transform_pose(
                        transformed_pose, self.static_transform)
                
                rospy.loginfo("marker {} is at target pose {} with respect to the robot base:".format(
                    marker.id, self.prettify(target_pose.pose.position)))


                observation = Observation()
                observation.id = marker.id
                observation.pose = target_pose

                new_msg.observations.append(observation)                    
                    
            except Exception, e:
                rospy.logerr(e)

        self.publisher.publish(new_msg)
        rospy.sleep(rospy.Duration(1))

    def prettify(self, point):
        return "({}, {}, {})".format(point.x,
            point.y, point.z)


def main_observer():
    rospy.init_node('observer_node')
    obs = Observer()

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, obs.callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    try:
        main_observer()
    except rospy.ROSInterruptException:
        pass

