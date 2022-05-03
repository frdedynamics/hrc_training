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


from Classes.main import Ui_Form as Form_0
from Classes.gui_node_class import GUInode

import roslaunch
import subprocess, time
from pathlib import Path
from os import popen, chdir

PKG_PATH = Path(QDir.currentPath()).parents[0]
gui_node = GUInode()


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

        self.rosTimer=QTimer()
        self.guiTimer=QTimer()
        self.rosTimer.timeout.connect(gui_node.update)
        self.guiTimer.timeout.connect(self.gui_update)

    
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

        # self.myProcess.readyReadStandardOutput.connect(self.write_process_output)
        self.show()


    def roscore_clicked(self):
        self.rosTimer.start(10)
        self.guiTimer.start(500)
        gui_node.init_subscribers_and_publishers()
        # self.start_single_roslaunch('/launch/gui.launch') # I need this
        self.RecordPlot.awindaButton.setEnabled(True)


    def awinda_clicked(self):
        #TODO
        # self.add_rosnode("hrc_training", "test2.py")
        self.RecordPlot.humanCalibrateButton.setEnabled(True)


    def humanCalibrate_clicked(self):
        #TODO
        # roslaunch_file = str(self.pkg_path)+'/launch/gui.launch'
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
        self.RecordPlot.awndalineEdit.setText(str(self.test_count))
        self.launch.start()

    def add_rosnode(self, pkg_name, node_name, args=None):
        node = roslaunch.core.Node(pkg_name, node_name)
        self.launch.launch(node)
        self.RecordPlot.awndalineEdit.setText(str(type(self.launch)))

    def stop_all_roslaunch(self):
        self.rosTimer.stop()
        self.guiTimer.stop()
        self.launch.shutdown()
        p = popen("rosnode kill /hrc_training_gui")
        p = popen("killall -9 roscore")
    
    def gui_update(self):
        self.RecordPlot.awndalineEdit.setText(str(gui_node.test_count))
        gui_node.update()
        gui_node.r.sleep()
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    # app.setStyleSheet(stylesheet)
    w = MainWindow()
    sys.exit(app.exec_())