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
        self.emg_sum_th = 3000
        self.elbow_height_th = 0.2
        

    def init_subscribers_and_publishers(self):
        self.sub_emg_sum = rospy.Subscriber('/emg_sum', Int64, queue_size=1)

    def set_params(self, emg_sum_th=3000, elbow_height_th=0.2):
        self.emg_sum_th = rospy.set_param('/emg_sum_th', emg_sum_th)
        self.elbow_height_th = rospy.set_param('/elbow_height_th', elbow_height_th)

    def data_logger_enabler(self):
        # TODO
        pass
        #     print "enable_logging"
        #     data_logger.enable_logging()

    def update(self):
        # self.test_count+=1
        self.r.sleep()

        # self.pub_p_hand.publish(self.p_hand)

        # implement this from  DH_game for Rviz visualization
        # try:
        #     result = self.score_client.call_server(self.task)
        #     print('The result is:', result)
        # except rospy.ROSInterruptException:
        #     print('Something went wrong:')

    def emg_sum_cb(self, msg):
        self.emg_sum = msg.data

    