#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pty import CHILD
from socket import timeout
from tokenize import String
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
import subprocess, time
from pathlib import Path
from os import popen, chdir

PKG_PATH = Path(QDir.currentPath()).parents[0]


class ROSProcess(QProcess):
    newROSout = pyqtSignal(str)

    def __init__(self, parent=None):
        super(ROSProcess, self).__init__(parent)

    def run(self):
        name = '/launch/gui.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [str(PKG_PATH)+name])
        self.launch.start()
        randomSample = "hello world"

        self.newROSout.emit(randomSample)


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
        self.myThread = ROSProcess(self)
        self.myThread.newROSout.connect(self.write_process_output)

        # self.myProcess.readyReadStandardOutput.connect(self.write_process_output)

    
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

        self.myThread.start()
        self.myThread.run()

        # self.start_single_roslaunch('/launch/gui.launch')

        # cmd = "ls"
        # proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        # user_list = str(proc.stdout.read())
        # self.RecordPlot.recordtextEdit.setText(user_list)


        ## Maybe better to write in a log file instead of only 
        # f = open("test.out", 'w')
        # sys.stdout = f
        # print "test"
        # f.close()

        # program = "roslaunch"
        # args = [str(self.pkg_path)+'/launch/gui.launch']
        # print(program)

        # self.myProcess.readyReadStandardOutput.connect(self.write_process_output)
        # self.myProcess.start(program, args)

        # self.myProcess.readAllStandardOutput().connect(self.write_process_output)


        # i=0
        # while i<5:
        #     time.sleep(1)
        #     self.RecordPlot.recordtextEdit.setText(str(i))
        #     i+=1

        # self.myProcess.kill()
        self.RecordPlot.recordtextEdit.setText("Started")

        # program.readyReadStandardOutput.connect(
        #     lambda process=program: self.write_process_output(process))

        self.RecordPlot.awindaButton.setEnabled(True)


    @pyqtSlot(str)
    def write_process_output(self, output):
        self.RecordPlot.recordtextEdit.setText(output)
        print("here")


    def awinda_clicked(self):
        #TODO
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
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [str(self.pkg_path)+name])
        # self.launch.start()

        # Read cmd output
        cmd = "roslaunch "+str(self.pkg_path)+name
        proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        user_list = str(proc.stdout.read())
        self.RecordPlot.recordtextEdit.setText(user_list)

        # Emit a signal there
        self.newSample.emit(user_list)

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