#!/usr/bin/env python3

"""
Gui's ROS related things got over this class. TEST for now

"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
# import Data.data_logger_module as data_logger

_CALIBRATION_TH = 60


class GUInode:
    def __init__(self, rate=30):
        """Initializes the GUI node.
        @param rate: ROS node spin rate"""
        rospy.init_node("hrc_gui_training", anonymous=False)
        self.r = rospy.Rate(rate)
        self.test_count = 0
        self.test_var = String()
        

    def init_subscribers_and_publishers(self):
        self.sub_test = rospy.Subscriber("chatter", String, self.test_cb)

    # def data_logger_enabler(self):
    #     print "enable_logging"
    #     data_logger.enable_logging()

    def update(self):
        self.test_count+=1
        print(self.test_count)
        print(self.test_var)
        self.r.sleep()

        # self.pub_p_hand.publish(self.p_hand)

    def test_cb(self, msg):
        self.test_var = msg
        print("here")

    