#!/usr/bin/env python3

import rospy, sys
import tf2_ros
from std_msgs.msg import String, Int16
from geometry_msgs.msg import Pose, Quaternion
from Classes.MarkerBasics import MarkerBasics

ref = 'human/base'

elbow_height_th = 0.0

elbow_left_height = 0.0
elbow_right_height = 0.0
force_mode = String()
hrc_state = String()
score_val = Int16()

def cb_elbow_left( msg):
    global elbow_left_height
    elbow_left_height = msg.position.y
    
def cb_elbow_right( msg):
    global elbow_right_height
    elbow_right_height = -msg.position.y

def cb_force_mode(msg):
    global force_mode
    force_mode = msg

def cb_hrc_state(msg):
    global hrc_state
    hrc_state = msg

def cb_score_val(msg):
    global score_val
    score_val = msg.data



def main():
    global elbow_right_height, elbow_left_height, elbow_height_th
    rospy.init_node('rviz_markers', anonymous=True)
    rate = rospy.Rate(100)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    sub_hrc_state = rospy.Subscriber('/hrc_state', String, cb_hrc_state)
    sub_colift_dir = rospy.Subscriber('/colift_dir', String, cb_force_mode)
    sub_score_val = rospy.Subscriber('/score_val', Int16, cb_score_val)
    sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, cb_elbow_left)
    sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, cb_elbow_right)

    left_arm_marker = MarkerBasics(topic_id="human/left_shoulder_", type="arm")
    right_arm_marker = MarkerBasics(topic_id="human/right_shoulder_", type="arm")
    score_marker = MarkerBasics(topic_id="score_", type="score")
    colift_dir_str_marker = MarkerBasics(topic_id="colift_dir_", type="regular_str")
    hrc_state_str_marker = MarkerBasics(topic_id="hrc_state_", type="regular_str")

    hrc_state_str_marker.change_position(0, 0, 1.6)
    hrc_state_str_marker.change_scale(0.8, 0.8, 0.8)
    hrc_state_str_marker.change_colour(1.0, 1.0, 1.0)

    while not rospy.has_param("/elbow_height_th"):
            print("no elbow parameter set")
            rospy.sleep(1)
            if rospy.is_shutdown():
                sys.exit()
    elbow_height_th = rospy.get_param("/elbow_height_th")
    print("ELBOW PARAM SET")

    while not rospy.has_param("/robot_move_started"):
        print("no /robot_move_started parameter set")
        rospy.sleep(1)
        if rospy.is_shutdown():
                sys.exit()

    print("/robot_move_started SET")

    while not rospy.has_param("/colift_set"):
        print("no /colift_set parameter set")
        force_mode.data = "u"
        rospy.sleep(1)
        if rospy.is_shutdown():
                sys.exit()

    print("/colift_set SET")

    

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

        print("elbow_left_height: ", elbow_left_height)
        print("elbow_right_height: ", elbow_right_height)
        print("force_mode: ", force_mode.data)
        print("elbow_height_th: ", elbow_height_th)
        

        if(elbow_left_height < elbow_height_th) and (elbow_right_height < elbow_height_th):
            if(force_mode.data == "s"):
                left_arm_marker.set_visible()
                left_arm_marker.change_colour(R=0, G=0, B=255)
                right_arm_marker.set_visible()
                right_arm_marker.change_colour(R=0, G=0, B=255)
                colift_dir_str_marker.update_str_marker("DOWN", R=0, G=0, B=255)
                print("DOWN")
            else: # force_mode.data = "d"
                left_arm_marker.set_visible(transparancy=0.2)
                right_arm_marker.set_visible(transparancy=0.2)            
                colift_dir_str_marker.update_str_marker("STOP", R=0, G=0, B=0)
                print("STOP")

        elif((elbow_left_height > elbow_height_th) and (elbow_right_height > elbow_height_th)):
            left_arm_marker.set_visible()
            left_arm_marker.change_colour(R=0, G=255, B=0)
            right_arm_marker.set_visible()
            right_arm_marker.change_colour(R=0, G=255, B=0)
            colift_dir_str_marker.update_str_marker("UP", R=0, G=255, B=0)
            print("UP")
        elif((elbow_left_height > elbow_height_th) and (elbow_right_height < elbow_height_th)):
            left_arm_marker.set_visible()
            left_arm_marker.change_colour(R=0, G=255, B=0)
            right_arm_marker.set_invisible()
            colift_dir_str_marker.update_str_marker("LEFT", R=0, G=255, B=0)
            print("LEFT")
        elif((elbow_right_height > elbow_height_th) and (elbow_left_height < elbow_height_th)):
            left_arm_marker.set_invisible()
            right_arm_marker.set_visible()
            right_arm_marker.change_colour(R=0, G=255, B=0)
            colift_dir_str_marker.update_str_marker("RIGHT", R=0, G=255, B=0)
            print("RIGHT")
        else:
            print("No force mode")
            left_arm_marker.set_invisible()
            right_arm_marker.set_invisible()

        score_marker.update_score_marker(score_val)
        hrc_state_str_marker.update_str_marker(hrc_state.data)
        left_arm_marker.marker_objectlisher.publish(left_arm_marker.marker_object)
        right_arm_marker.marker_objectlisher.publish(right_arm_marker.marker_object)
        score_marker.marker_objectlisher.publish(score_marker.marker_object)
        colift_dir_str_marker.marker_objectlisher.publish(colift_dir_str_marker.marker_object)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass