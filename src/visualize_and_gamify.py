#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Vector3
import tf.transformations

from Classes.MarkerBasics import MarkerBasics
import rospkg, sys
imu_pkg = rospkg.RosPack()
imu_pkg_path = imu_pkg.get_path('imu_human_pkg')+"/src/Classes"
# print(imu_pkg_path)
sys.path.insert(0, imu_pkg_path)
import Kinematics_with_Quaternions as kinematic

ref = 'human/base'

elbow_left_height = Pose()
elbow_right_height = Pose()

def cb_elbow_left(msg):
    global elbow_left_height
    elbow_left_height = msg.position.y

    
def cb_elbow_right(msg):
    global elbow_right_height
    elbow_right_height = -msg.position.y

def main():
    rospy.init_node('rviz_markers', anonymous=True)
    rate = rospy.Rate(100)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, cb_elbow_left)
    sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, cb_elbow_right)

    left_arm_marker = MarkerBasics(topic_id="human/left_shoulder")
    right_arm_marker = MarkerBasics(topic_id="human/right_shoulder")

    # right_bias = Quaternion([0, 0, 0.7, 0.7])
    left_marker_orientation = Quaternion(0.0, 0.0, 0, 0.1)

    try:
        elbow_height_th = rospy.get_param("/elbow_height_th")
        print("Elbow param set: ", elbow_height_th)
    except KeyError as e:
        elbow_height_th = 0.2
        print("Elbow parameter is not set by GUI.")
        print(e)
    
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

        if elbow_left_height >= elbow_height_th:
            left_arm_marker.set_visible()
        else:
            left_arm_marker.set_invisible()

        if elbow_right_height >= elbow_height_th:
            right_arm_marker.set_visible()
        else:
            right_arm_marker.set_invisible()


        left_arm_marker.marker_objectlisher.publish(left_arm_marker.marker_object)
        right_arm_marker.marker_objectlisher.publish(right_arm_marker.marker_object)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass