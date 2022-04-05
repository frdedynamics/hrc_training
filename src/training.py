#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pty import CHILD
from socket import timeout
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import pandas as pd
from math import pi

import sys, random
from shutil import copy
from sensor_msgs.msg import JointState

from Classes.main import Ui_Form as Form_0

import roslaunch, rospy
import subprocess
from pathlib import Path
from os import popen


class RecordPlotWindow(QWidget, Form_0):
    def __init__(self, parent=None):
        super(RecordPlotWindow, self).__init__(parent)
        self.setupUi(self)
        

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.resize(1000, 767)
        self.startRecordPlotWindow()
        self.pkg_path = Path(QDir.currentPath()).parents[0]

    
    def startRecordPlotWindow(self):
        self.RecordPlot = RecordPlotWindow(self)
        self.setCentralWidget(self.RecordPlot)
        self.setWindowTitle("HRC Training")
        self.RecordPlot.roscoreButton.clicked.connect(self.roscore_clicked)
        self.RecordPlot.buttonBox.rejected.connect(self.stop_all_roslaunch)
        self.show()


    def roscore_clicked(self):
        self.start_single_roslaunch('/launch/gui.launch')


    def start_single_roslaunch(self, name):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [str(self.pkg_path)+name])
        self.launch.start()

    def stop_all_roslaunch(self):
        self.launch.shutdown()
        p = popen("killall -9 roscore")


    # def start_roslaunch(self):
    #     rospy.init_node('robot_designer', anonymous=False)
    #     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #     roslaunch.configure_logging(uuid)
    #     self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/gizem/catkin_ws/src/dh_game/launch/mrm.launch"])
    #     self.launch.start()
    #     self.sub_joint_states = rospy.Subscriber("/joint_states", JointState, self.cb_joint_states)
    #     self.pub_dh_table_theta = rospy.Publisher("/dh_table_theta", JointState, queue_size=10)
    #     self.pub_dh_table_s = rospy.Publisher("/dh_table_s", JointState, queue_size=10)
    #     self.pub_dh_table_d = rospy.Publisher("/dh_table_d", JointState, queue_size=10)
    #     self.pub_dh_table_alpha = rospy.Publisher("/dh_table_alpha", JointState, queue_size=10)
    #     rospy.loginfo("started")

    #     print(self.TasksTool.task_selection)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    # app.setStyleSheet(stylesheet)
    w = MainWindow()
    sys.exit(app.exec_())