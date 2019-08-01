#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import tf2_geometry_msgs
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from observer_node.msg import Observation, Observations


REFERENCE_TAG_ID = 0
ROBOT_TAG_ID = 1
ORIGIN_FRAME = "origin"
ROBOT_BASE_FRAME = "m1n6s200_link_base"

class Observer:

    def __init__(self, **args):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.broadcaster_static = tf2_ros.StaticTransformBroadcaster()
        self.origin_marker_frame = "ar_marker_{}".format(REFERENCE_TAG_ID)
        self.camera_frame = "kinect2_link"

        self.publisher = rospy.Publisher('observer_node', Observations, queue_size=1)
        rospy.loginfo("Initialized {}".format(self))
    
    def callback(self, msg):
        """
        This method serves as the callback for the ar_pose_marker topic to which the observer
        is subscribed. This callback method will tranform the detected ar tags and publish them
        on the /observer_node topic.
        """
        rospy.loginfo("Transforming markers {}".format(self))
        new_msg = Observations()
        new_msg.header = msg.header
        self.transform = self.tf_buffer.lookup_transform(
                self.origin_marker_frame, self.camera_frame, rospy.Time(0), rospy.Duration(5))

        for marker in msg.markers:
            if marker.id in [ROBOT_TAG_ID]:
                continue #The robot tag is not needed right now
            try: 
                transformed_pose = tf2_geometry_msgs.do_transform_pose(
                        marker.pose, self.transform)
                observation = Observation()
                observation.id = marker.id
                observation.pose = transformed_pose
                rospy.loginfo("marker {} transformed_pose {}".format(
                    marker.id, self.prettify(transformed_pose)))
                new_msg.observations.append(observation)                    
                    
            except Exception, e:
                rospy.logerr(e)

        self.publisher.publish(new_msg)
        rospy.sleep(rospy.Duration(1))

    def prettify(self, pose):
        return "({}, {}, {})".format(pose.pose.position.x,
            pose.pose.position.y, pose.pose.position.z)


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

