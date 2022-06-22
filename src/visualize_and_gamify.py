#!/usr/bin/env python3

import rospy
import tf2_ros
from std_msgs.msg import String, Int16
from geometry_msgs.msg import Pose, Quaternion
from Classes.MarkerBasics import MarkerBasics

ref = 'human/base'

force_mode = String()
score_val = Int16()

def cb_force_mode(msg):
    global force_mode
    force_mode = msg.data

def cb_score_val(msg):
    global score_val
    score_val = msg.data



def main():
    rospy.init_node('rviz_markers', anonymous=True)
    rate = rospy.Rate(100)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    sub_elbow_left = rospy.Subscriber('/force_mode', String, cb_force_mode)
    sub_score_val = rospy.Subscriber('/score_val', Int16, cb_score_val)

    left_arm_marker = MarkerBasics(topic_id="human/left_shoulder_", type="arm")
    right_arm_marker = MarkerBasics(topic_id="human/right_shoulder_", type="arm")
    score_marker = MarkerBasics(topic_id="score_", type="score")


    while not rospy.is_shutdown():
        try:
            left_shoulder_trans = tfBuffer.lookup_transform(ref, 'human/left_shoulder', rospy.Time())
            right_shoulder_trans = tfBuffer.lookup_transform(ref, 'human/right_shoulder', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        left_arm_marker.marker_object.pose.position = left_shoulder_trans.transform.translation
        left_arm_marker.marker_object.pose.orientation = left_shoulder_trans.transform.rotation

        right_arm_marker.marker_object.pose.position = right_shoulder_trans.transform.translation
        right_arm_marker.marker_object.pose.orientation = right_shoulder_trans.transform.rotation

        if force_mode == 'up':
            left_arm_marker.set_visible()
            left_arm_marker.change_colour(R=0, G=255, B=0)
            right_arm_marker.set_visible()
            right_arm_marker.change_colour(R=0, G=255, B=0)
        elif force_mode == 'down':
            left_arm_marker.set_visible()
            left_arm_marker.change_colour(R=0, G=0, B=255)
            right_arm_marker.set_visible()
            right_arm_marker.change_colour(R=0, G=0, B=255)
        elif force_mode == 'left':
            left_arm_marker.set_visible()
            left_arm_marker.change_colour(R=0, G=255, B=0)
            right_arm_marker.set_invisible()
        elif force_mode == 'right':
            left_arm_marker.set_invisible()
            right_arm_marker.set_visible()
            right_arm_marker.change_colour(R=0, G=255, B=0)
        elif force_mode == 'null':
            left_arm_marker.set_visible(transparancy=0.2)
            right_arm_marker.set_visible(transparancy=0.2)            
        else:
            print("No force mode")
            left_arm_marker.set_invisible()
            right_arm_marker.set_invisible()

        score_marker.update_score_marker(score_val)
        left_arm_marker.marker_objectlisher.publish(left_arm_marker.marker_object)
        right_arm_marker.marker_objectlisher.publish(right_arm_marker.marker_object)
        score_marker.marker_objectlisher.publish(score_marker.marker_object)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass