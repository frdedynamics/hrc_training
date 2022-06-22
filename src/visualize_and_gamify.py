#!/usr/bin/env python3

import rospy
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion
from Classes.MarkerBasics import MarkerBasics

ref = 'human/base'

force_mode = String()

def cb_force_mode(msg):
    global force_mode
    force_mode = msg.data


def main():
    rospy.init_node('rviz_markers', anonymous=True)
    rate = rospy.Rate(100)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    sub_elbow_left = rospy.Subscriber('/force_mode', String, cb_force_mode)

    left_arm_marker = MarkerBasics(topic_id="human/left_shoulder", type="arm")
    right_arm_marker = MarkerBasics(topic_id="human/right_shoulder", type="arm")
    score_marker = MarkerBasics(topic_id="score_marker", type="score")


    while not rospy.is_shutdown():
        try:
            left_shoulder_trans = tfBuffer.lookup_transform(ref, 'human/left_shoulder_0', rospy.Time())
            right_shoulder_trans = tfBuffer.lookup_transform(ref, 'human/right_shoulder', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        left_arm_marker.marker_object.pose.position = left_shoulder_trans.transform.translation
        left_arm_marker.marker_object.pose.orientation = left_shoulder_trans.transform.rotation

        right_arm_marker.marker_object.pose.position = right_shoulder_trans.transform.translation
        right_arm_marker.marker_object.pose.orientation = right_shoulder_trans.transform.rotation

        if force_mode.data == 'up':
            left_arm_marker.set_visible()
            right_arm_marker.set_visible()
        elif force_mode.data == 'down':
            left_arm_marker.set_invisible()
            right_arm_marker.set_invisible()
        elif force_mode.data == 'left':
            left_arm_marker.set_visible()
            right_arm_marker.set_invisible()
        elif force_mode.data == 'right':
            left_arm_marker.set_invisible()
            right_arm_marker.set_visible()
        if force_mode.data == 'down':
            # TODO: Also change color to red maybe?
            left_arm_marker.set_visible()
            right_arm_marker.set_visible()
        else:
            right_arm_marker.set_invisible()


        left_arm_marker.marker_objectlisher.publish(left_arm_marker.marker_object)
        right_arm_marker.marker_objectlisher.publish(right_arm_marker.marker_object)
        score_marker.marker_objectlisher.publish(score_marker.marker_object)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass