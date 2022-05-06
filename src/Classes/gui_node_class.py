#!/usr/bin/env python3

"""
Gui's ROS related things got over this class. TEST for now

"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int64
# import Data.data_logger_module as data_logger

_CALIBRATION_TH = 60


class GUInode:
    def __init__(self, rate=30):
        """Initializes the GUI node.
        @param rate: ROS node spin rate"""
        rospy.init_node("hrc_gui_training", anonymous=False)
        self.r = rospy.Rate(rate)
        self.emg_sum = Int64()
        

    def init_subscribers_and_publishers(self):
        self.sub_emg_sum = rospy.Subscriber('/emg_sum', Int64, queue_size=1)

    def data_logger_enabler(self):
        # TODO
        pass
        #     print "enable_logging"
        #     data_logger.enable_logging()

    def update(self):
        # self.test_count+=1
        self.r.sleep()

        # self.pub_p_hand.publish(self.p_hand)

    def emg_sum_cb(self, msg):
        self.emg_sum = msg.data

    