#!/usr/bin/env python3

"""
Gui's ROS related things got over this class. TEST for now

"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int64, Int16
from geometry_msgs.msg import Pose
# import Data.data_logger_module as data_logger

_CALIBRATION_TH = 60


class GUInode:
    def __init__(self, rate=30):
        """Initializes the GUI node.
        @param rate: ROS node spin rate"""
        rospy.init_node("hrc_gui_training", anonymous=False)
        self.r = rospy.Rate(rate)
        self.emg_sum = Int64()
        self.emg_sum_th = 3000
        self.right_elbow_current = 0
        self.left_elbow_current = 0
        self.elbow_height_th = 0.2
        self.score_val = 600
        

    def init_subscribers_and_publishers(self):
        # EMG sum
        self.sub_emg_sum = rospy.Subscriber('/emg_sum', Int64, self.emg_sum_cb)
        # 2 elbow heights
        self.sub_right_elbow = rospy.Subscriber('/elbow_right', Pose, self.right_elbow_cb)
        self.sub_left_elbow = rospy.Subscriber('/left_right', Pose, self.left_elbow_cb)
        # 3 chest-to-wrist poses
        # merged hands 
        # HRC states
        # Colift states
        # TCP pose
        # TCP force
        # Table acc and ori
        # Buttons states

        ## PUBLISH
        # Score
        self.pub_score_val = rospy.Publisher('/score_val', Int16, queue_size=1)
        # Merged hands 2 -- to be used in colift state


    def set_params(self, emg_sum_th=3000, elbow_height_th=0.2):
        self.emg_sum_th = rospy.set_param('/emg_sum_th', emg_sum_th)
        self.elbow_height_th = rospy.set_param('/elbow_height_th', elbow_height_th)


    def data_logger_enabler(self):
        # TODO
        pass
        #     print "enable_logging"
        #     data_logger.enable_logging()

    def score_calculator(self):
        # Start 600
        self.score_val = 600
        # -1 each second
        # +60 each button
        # update score marker

    def update(self):
        # self.test_count+=1
        self.score_calculator()
        self.r.sleep()

        # self.pub_p_hand.publish(self.p_hand)
        self.pub_score_val.publish(self.score_val)

        # implement this from  DH_game for Rviz visualization
        # try:
        #     result = self.score_client.call_server(self.task)
        #     print('The result is:', result)
        # except rospy.ROSInterruptException:
        #     print('Something went wrong:')

    def emg_sum_cb(self, msg):
        self.emg_sum = msg.data


    def right_elbow_cb(self, msg):
        self.right_elbow_current = msg.position.y

    def left_elbow_cb(self, msg):
        self.left_elbow_current = msg.position.y

    