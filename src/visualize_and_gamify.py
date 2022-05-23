#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion
import tf.transformations

from Classes.MarkerBasics import MarkerBasics
import rospkg, sys
imu_pkg = rospkg.RosPack()
imu_pkg_path = imu_pkg.get_path('imu_human_pkg')+"/src/Classes"
# print(imu_pkg_path)
sys.path.insert(0, imu_pkg_path)
import Kinematics_with_Quaternions as kinematic

ref = 'base_link'

def main():
    rospy.init_node('rviz_markers', anonymous=True)
    rate = rospy.Rate(100)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    left_arm_marker = MarkerBasics(topic_id="human/left_shoulder")
    right_arm_marker = MarkerBasics(topic_id="human/right_shoulder")

    # right_bias = Quaternion([0, 0, 0.7, 0.7])
    # left_bias = Quaternion([0, 0, 0.7, 0.7])
    
    while not rospy.is_shutdown():
        try:
            left_shoulder_trans = tfBuffer.lookup_transform(ref, 'human/left_shoulder', rospy.Time())
            right_shoulder_trans = tfBuffer.lookup_transform(ref, 'human/right_shoulder', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        left_arm_marker.marker_object.pose.position = left_shoulder_trans.transform.translation
        left_arm_marker.marker_object.pose.orientation = left_shoulder_trans.transform.rotation
        # left_arm_marker.marker_object.pose.orientation = tf.transformations.quaternion_multiply( right_bias,[left_shoulder_trans.transform.rotation.x, left_shoulder_trans.transform.rotation.y, left_shoulder_trans.transform.rotation.z, left_shoulder_trans.transform.rotation.w])
        # print(left_shoulder_trans.transform.rotation.x)
        # print(left_arm_marker.marker_object.pose.orientation.x)
        # print(type(left_shoulder_trans.transform.rotation))
        # left_arm_marker.marker_object.pose.orientation = kinematic.kinematic.q_multiply(left_shoulder_trans.transform.rotation, left_bias)
        # print(left_arm_marker.marker_object.pose.orientation)
        right_arm_marker.marker_object.pose.position = right_shoulder_trans.transform.translation
        right_arm_marker.marker_object.pose.orientation = right_shoulder_trans.transform.rotation

        left_arm_marker.marker_objectlisher.publish(left_arm_marker.marker_object)
        right_arm_marker.marker_objectlisher.publish(right_arm_marker.marker_object)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass