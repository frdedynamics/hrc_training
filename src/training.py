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
import os, sys, subprocess


class RecordPlotWindow(QWidget, Form_0):
    def __init__(self, parent=None):
        super(RecordPlotWindow, self).__init__(parent)
        self.setupUi(self)
        

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.resize(1000, 767)
        self.startRecordPlotWindow()
        self.dof_set_flag = False
        self.joint_states = []
        self.joints = []
        self.links = []
        self.dh_items_theta = JointState()
        self.dh_items_s = JointState()
        self.dh_items_d = JointState()
        self.dh_items_alpha = JointState()

    
    def startRecordPlotWindow(self):
        self.RecordPlot = RecordPlotWindow(self)
        self.setCentralWidget(self.RecordPlot)
        self.setWindowTitle("HRC Training")
        self.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    # app.setStyleSheet(stylesheet)
    w = MainWindow()
    sys.exit(app.exec_())