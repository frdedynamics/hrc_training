#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from builtins import AttributeError
from pty import CHILD
from socket import timeout
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import pandas as pd
from math import pi

import sys
from time import sleep
from os import chdir

from Classes.main import Ui_Form as Form_0
from Classes.gui_node_class import GUInode
from Classes.new_user import Ui_Dialog as NewUserDialog

import Classes.randomIDcreator as randomIDcreator

import roslaunch
import subprocess, time
from pathlib import Path
from os import popen, chdir

PKG_PATH = Path(QDir.currentPath()).parents[0]
DATA_PATH = '/home/gizem/Insync/giat@hvl.no/Onedrive/HVL/Human_Experiments/data/'


class RecordPlotWindow(QWidget, Form_0):
    def __init__(self, parent=None):
        super(RecordPlotWindow, self).__init__(parent)
        self.setupUi(self)


class NewUser(QDialog, NewUserDialog):
    def __init__(self, parent=None):
        super(NewUser, self).__init__(parent)
        self.setupUi(self)
        self.userID = 0

    def create_random_ID(self, name, height, arm_length, left_handed):
        self.userID = randomIDcreator.main(name, height, arm_length, left_handed)
        self.IDlabel.setText(str(self.userID))
        

class MainWindow(QMainWindow, Form_0):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.resize(1000, 767)
        self.startRecordPlotWindow()
        self.pkg_path = Path(QDir.currentPath()).parents[0]  ##(?) QDir() instead? check later
        self.launch = roslaunch.scriptapi.ROSLaunch()

        self.ros_node = GUInode()
        self.rosTimer=QTimer()
        self.guiTimer=QTimer()
        self.rosTimer.timeout.connect(self.ros_node.update)
        self.guiTimer.timeout.connect(self.gui_update)

        self.human_proc = None
        self.myo_proc = None
        self.urdt_proc = None
        self.gripper_proc = None

        self.user_path = DATA_PATH


    def open_new_user_dialog(self):
        self.Dialog = QDialog()
        self.NewUserTool = NewUser(self)
        self.NewUserTool.setupUi(self.Dialog)

        # call randomIDcreator script here
        self.NewUserTool.createButton.clicked.connect(lambda: self.NewUserTool.create_random_ID(
                                                                self.NewUserTool.nameLineEdit.text(),
                                                                self.NewUserTool.heightLineEdit.text(),
                                                                self.NewUserTool.armLengthLineEdit.text(),
                                                                self.NewUserTool.leftHandcheckBox.isChecked()))
        # a = randomIDcreator()
        # self.NewUserTool.IDlabel.setText(a)

        self.Dialog.show()
        self.Dialog.exec_()

    
    def startRecordPlotWindow(self):
        self.RecordPlot = RecordPlotWindow(self)
        self.setCentralWidget(self.RecordPlot)
        self.setWindowTitle("HRC Training")

        self.RecordPlot.awindaButton.setEnabled(False)
        self.RecordPlot.humanCalibrateButton.setEnabled(False)
        self.RecordPlot.humanJointResetButton.setEnabled(False)
        self.RecordPlot.emgResetButton.setEnabled(False)
        self.RecordPlot.humanInitiateButton.setEnabled(True)
        self.RecordPlot.gripperInitiateButton.setEnabled(False)
        self.RecordPlot.robotMoveButton.setEnabled(False)

        self.RecordPlot.roscoreButton.clicked.connect(self.roscore_clicked)
        self.RecordPlot.awindaButton.clicked.connect(self.awinda_clicked)
        self.RecordPlot.humanCalibrateButton.clicked.connect(self.humanCalibrate_clicked)
        self.RecordPlot.humanJointResetButton.clicked.connect(self.humanJointReset_clicked)
        self.RecordPlot.emgResetButton.clicked.connect(self.emgReset_clicked)
        self.RecordPlot.humanInitiateButton.clicked.connect(self.humanInitiate_clicked)
        self.RecordPlot.gripperInitiateButton.clicked.connect(self.gripperInitiate_clicked)
        self.RecordPlot.robotMoveButton.clicked.connect(self.robotMove_clicked)
        self.RecordPlot.buttonBox.clicked.connect(self.stop_all_roslaunch)

        self.RecordPlot.newUserButton.clicked.connect(self.open_new_user_dialog)

        self.RecordPlot.recordtextEdit.setText("WELCOME to HVL Robotics HRC bla bla")
        self.RecordPlot.recordtextEdit.verticalScrollBar().setValue(self.RecordPlot.recordtextEdit.verticalScrollBar().maximum())

        self.show()


    def roscore_clicked(self):
        self.rosTimer.start(10)
        self.guiTimer.start(500)
        self.ros_node.init_subscribers_and_publishers()
        self.start_single_roslaunch('/launch/gui.launch') # I need this to set a UUID for later added nodes
        
        self.RecordPlot.awindaButton.setEnabled(True)


    def awinda_clicked(self):
        self.add_rosnode("awindamonitor", "awindamonitor", "awindamonitor")
        self.RecordPlot.humanCalibrateButton.setEnabled(True)
        

    def humanCalibrate_clicked(self):
        # self.add_rosnode(node_name="myo-rawNode.py", pkg_name="ros_myo")
        # self.add_rosnode("world_to_myo.py", "arm_motion_controller_py3")
        # self.add_rosnode("rviz", "rviz", args="-d $(find arm_motion_controller_py3)/launch/config/config_with_myo.rviz")
        self.human_proc = subprocess.Popen(["sh", "../sh/human.sh"]) ## this will halt the system. You will use Popen: https://stackoverflow.com/questions/16855642/execute-a-shell-script-from-python-subprocess
        sleep(1.0)
        self.myo_proc = subprocess.Popen(["sh", "../sh/myo.sh"]) 

        self.RecordPlot.emgResetButton.setEnabled(True)
        self.RecordPlot.humanInitiateButton.setEnabled(True)
        self.RecordPlot.humanJointResetButton.setEnabled(True)

    
    def humanJointReset_clicked(self):
        self.RecordPlot.humanJointResetButton.setEnabled(False)
        self.RecordPlot.recordtextEdit.append("Human joints are resetting")
        subprocess.Popen(["rosnode", "kill", "/imu_subscriber_node"])
        sleep(1.0)
        self.add_rosnode("arm_motion_controller_py3", "imu_subscriber_node.py", "imu_subscriber_node")
        self.RecordPlot.recordtextEdit.append("Human joint reset DONE")
        self.RecordPlot.humanJointResetButton.setEnabled(True)

    def emgReset_clicked(self):
        self.RecordPlot.emgResetButton.setEnabled(False)
        self.RecordPlot.recordtextEdit.append("EMG resetting")
        subprocess.Popen(["rosnode", "kill", "/myo_raw", "/emg_to_gripper", "/world_to_myo_tf_publisher"])
        sleep(2.0)
        subprocess.Popen(["sh", "../sh/myo.sh"])
        self.RecordPlot.recordtextEdit.append("EMG reset DONE")
        self.RecordPlot.emgResetButton.setEnabled(True)
        # TODO: check if EMG sum is changing over time

        

    def humanInitiate_clicked(self):
        self.add_rosnode("arm_motion_controller_py3", "wrist_to_robot_2arms.py", "wrist_to_robot_2arms")
        self.RecordPlot.recordtextEdit.append("-----CALIBRATION STARTED-----")
        self.RecordPlot.recordtextEdit.append("Move to initial arm poses in 3 seconds...")
        self.RecordPlot.recordtextEdit.append("3 seconds...")
        self.RecordPlot.recordtextEdit.verticalScrollBar().setValue(self.RecordPlot.recordtextEdit.verticalScrollBar().maximum())
        self.RecordPlot.recordtextEdit.repaint()
        sleep(1)
        self.RecordPlot.recordtextEdit.append("2 seconds...")
        self.RecordPlot.recordtextEdit.verticalScrollBar().setValue(self.RecordPlot.recordtextEdit.verticalScrollBar().maximum())
        self.RecordPlot.recordtextEdit.repaint()
        sleep(1)
        self.RecordPlot.recordtextEdit.append("1 second...")
        self.RecordPlot.recordtextEdit.verticalScrollBar().setValue(self.RecordPlot.recordtextEdit.verticalScrollBar().maximum())
        self.RecordPlot.recordtextEdit.repaint()
        sleep(1)
        self.RecordPlot.recordtextEdit.append("Initial arm poses registered")
        self.RecordPlot.recordtextEdit.verticalScrollBar().setValue(self.RecordPlot.recordtextEdit.verticalScrollBar().maximum())
        self.RecordPlot.recordtextEdit.repaint()

        # TODO: set elbow_height_th, emg_sum, hand_dominance parameters
        if (len(self.RecordPlot.elbowTresholdLineEdit.text())>0 and self.RecordPlot.emgTresholdLabel.text()>0):
            elbow_th = int(self.RecordPlot.elbowTresholdLineEdit.text())
            emg_th = int(self.RecordPlot.emgTresholdLabel.text())
            self.ros_node.set_params(elbow_height_th=elbow_th, emg_sum_th=emg_th)

            self.RecordPlot.gripperInitiateButton.setEnabled(True)
        else:
            print("Elbow and EMG calibrations are not completed")


    def gripperInitiate_clicked(self):
        gripper_process = subprocess.Popen(["rospack", "find", "robotiq_urcap_control"], stdout=subprocess.PIPE)
        gripper_ros_path = str(gripper_process.stdout.readline())[2:-3]
        chdir(gripper_ros_path+'/src/')
        # gripper_process = subprocess.Popen(["ls"], stdout=subprocess.PIPE)
        # stdout = str(gripper_process.stdout.readlines())
        # print('STDOUT:{}'.format(stdout))
        # sys.exit()
        self.gripper_proc = subprocess.Popen(["python3", "robotic_urcap_ctrl_py3.py", "172.31.1.144"]) # robot ip
        self.add_rosnode("ros_magic_pkg", "map_robotiq_to_topic_bool.py", "map_robotiq_to_topic_bool")
        self.RecordPlot.robotMoveButton.setEnabled(True)


    def robotMove_clicked(self):
        self.urdt_proc = subprocess.Popen(["/home/gizem/venv/venv-ur/bin/python3.8", "/home/gizem/catkin_ws/src/arm_motion_controller_py3/src/robot_move_node.py"])  ## For other PCs or multiple PCs this needs to be changes. Not modular.


    def start_single_roslaunch(self, name):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [str(self.pkg_path)+name])
        self.RecordPlot.recordtextEdit.append(name+" started")
        self.launch.start()

    def add_rosnode(self, pkg_name, node_name, name, args=None, respawn=True):
        node = roslaunch.core.Node(pkg_name, node_name, name, args)
        self.launch.launch(node)
        self.RecordPlot.recordtextEdit.append(node_name+" in "+pkg_name+" is started")

    def stop_all_roslaunch(self):
        try:
            self.human_proc.kill()
            self.myo_proc.kill()
            self.urdt_proc.kill()
            self.gripper_proc.kill()
        except AttributeError as e:
            pass
        p_kill1 = subprocess.Popen(["rosnode", "kill", "-a"]) # not sure if I need a return object. Keep it for now
        p_kill2 = subprocess.Popen(["pkill", "-9", "ros"])
        # TODO: kill Rviz manually
        self.rosTimer.stop()
        self.guiTimer.stop()
        # self.launch.shutdown()


    def gui_update(self):
        self.ros_node.update()
        self.ros_node.r.sleep()
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    # app.setStyleSheet(stylesheet)
    w = MainWindow()
    sys.exit(app.exec_())