#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pty import CHILD
from socket import timeout
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import pandas as pd
from math import pi

import sys

from Classes.main import Ui_Form as Form_0
from Classes.gui_node_class import GUInode

import roslaunch
import subprocess, time
from pathlib import Path
from os import popen, chdir

PKG_PATH = Path(QDir.currentPath()).parents[0]


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
        self.launch = roslaunch.scriptapi.ROSLaunch()

        self.ros_node = GUInode()
        self.rosTimer=QTimer()
        self.guiTimer=QTimer()
        self.rosTimer.timeout.connect(self.ros_node.update)
        self.guiTimer.timeout.connect(self.gui_update)

        self.human_with_myo_proc = None

    
    def startRecordPlotWindow(self):
        self.RecordPlot = RecordPlotWindow(self)
        self.setCentralWidget(self.RecordPlot)
        self.setWindowTitle("HRC Training")

        self.RecordPlot.awindaButton.setEnabled(False)
        self.RecordPlot.humanCalibrateButton.setEnabled(False)
        self.RecordPlot.emgResetButton.setEnabled(False)
        self.RecordPlot.humanInitiateButton.setEnabled(False)
        self.RecordPlot.gripperInitiateButton.setEnabled(False)
        self.RecordPlot.robotMoveButton.setEnabled(False)

        self.RecordPlot.roscoreButton.clicked.connect(self.roscore_clicked)
        self.RecordPlot.awindaButton.clicked.connect(self.awinda_clicked)
        self.RecordPlot.humanCalibrateButton.clicked.connect(self.humanCalibrate_clicked)
        self.RecordPlot.emgResetButton.clicked.connect(self.emgReset_clicked)
        self.RecordPlot.humanInitiateButton.clicked.connect(self.humanInitiate_clicked)
        self.RecordPlot.gripperInitiateButton.clicked.connect(self.gripperInitiate_clicked)
        self.RecordPlot.robotMoveButton.clicked.connect(self.robotMove_clicked)
        self.RecordPlot.buttonBox.rejected.connect(self.stop_all_roslaunch)

        self.RecordPlot.recordtextEdit.setText("WELCOME to HVL Robotics HRC bla bla")

        self.show()


    def roscore_clicked(self):
        self.rosTimer.start(10)
        self.guiTimer.start(500)
        self.ros_node.init_subscribers_and_publishers()
        self.start_single_roslaunch('/launch/gui.launch') # I need this to set a UUID for later added nodes
        
        self.RecordPlot.awindaButton.setEnabled(True)


    def awinda_clicked(self):
        self.human_with_myo_proc = subprocess.Popen(["sh", "../sh/human_with_myo.sh"], stdin=subprocess.PIPE)
        # self.add_rosnode("awindamonitor", "awindamonitor")
        self.RecordPlot.humanCalibrateButton.setEnabled(True)
        

    def humanCalibrate_clicked(self):
        # self.add_rosnode(node_name="myo-rawNode.py", pkg_name="ros_myo")
        # self.add_rosnode("world_to_myo.py", "arm_motion_controller_py3")
        # self.add_rosnode("rviz", "rviz", args="-d $(find arm_motion_controller_py3)/launch/config/config_with_myo.rviz")
        # subprocess.run(["sh", "../sh/human_with_myo.sh"]) // this will halt the system. You will use Popen: https://stackoverflow.com/questions/16855642/execute-a-shell-script-from-python-subprocess
        p_kill = subprocess.Popen(["pkill", "-9", "ros"])



        self.RecordPlot.emgResetButton.setEnabled(True)
        self.RecordPlot.humanInitiateButton.setEnabled(True)


    def emgReset_clicked(self):
        pass
        

    def humanInitiate_clicked(self):
        #TODO
        self.RecordPlot.gripperInitiateButton.setEnabled(True)


    def gripperInitiate_clicked(self):
        #TODO
        self.RecordPlot.robotMoveButton.setEnabled(True)


    def robotMove_clicked(self):
        pass


    def start_single_roslaunch(self, name):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [str(self.pkg_path)+name])
        self.RecordPlot.recordtextEdit.append(name+" started")
        self.launch.start()

    def add_rosnode(self, pkg_name, node_name, args=None):
        node = roslaunch.core.Node(pkg_name, node_name, args)
        self.launch.launch(node)
        self.RecordPlot.recordtextEdit.append(node_name+" in "+pkg_name+" is started")

    def stop_all_roslaunch(self):
        p_kill = subprocess.Popen(["pkill", "-9", "ros"])
        self.rosTimer.stop()
        self.guiTimer.stop()
        # self.launch.shutdown()
        self.human_with_myo_proc.kill()
        # p_node_kill = popen("rosnode kill /hrc_training_gui")
        # p_kill = popen("killall -9 roscore") ## does not kill all roscore
        p_kill = subprocess.Popen(["pkill", "-9", "ros"])

    def gui_update(self):
        self.RecordPlot.awndalineEdit.setText(str(self.ros_node.test_count))
        self.ros_node.update()
        self.ros_node.r.sleep()
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    # app.setStyleSheet(stylesheet)
    w = MainWindow()
    sys.exit(app.exec_())