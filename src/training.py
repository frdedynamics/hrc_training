#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import pandas as pd
from math import pi

import sys
import re # for split string with multiple delimiters
from time import sleep, time
from os import chdir

from Classes.main import Ui_Form as Form_0
from Classes.gui_node_class import GUInode
from Classes.new_user import Ui_Dialog as NewUserDialog

import Classes.randomIDcreator as randomIDcreator

import roslaunch
import signal, logging
import subprocess, time, datetime
from pathlib import Path
from os import popen, chdir, mkdir, getcwd

import rospy

PKG_PATH = Path(QDir.currentPath()).parents[0]
DATA_PATH = '/home/gizem/Insync/giat@hvl.no/Onedrive/HVL/Human_Experiments/data/'
SELECTED_ID = 0



class RecordPlotWindow(QWidget, Form_0):
    def __init__(self, parent=None):
        super(RecordPlotWindow, self).__init__(parent)

        self.trial_no = 0
        self.selectedID = 0
        self.users_list = []
        self.setupUi(self)


class NewUser(QDialog, NewUserDialog):
    def __init__(self, parent=None):
        super(NewUser, self).__init__(parent)
        self.setupUi(self)
        self.userID = 0

    def create_random_ID(self, name, height, arm_length, left_handed):
        self.userID = randomIDcreator.main(name, height, arm_length, left_handed)
        self.IDlabel.setText(str(self.userID))
        SELECTED_ID = self.userID
    
    # def create_user_folder(self):
    #     new_folder = DATA_PATH+"users/"+str(self.userID)
    #     try:
    #         mkdir(new_folder)
    #     except FileExistsError:
    #         pass
    #     print("New user is ready with ID number create_user_folder: ", self.userID)


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
        self.thresholdsTimer=QTimer()
        self.rosTimer.timeout.connect(self.ros_node.update)
        self.guiTimer.timeout.connect(self.gui_update)
        self.thresholdsTimer.timeout.connect(self.measure_thresholds_update)

        self.logging_started_flag = False

        self.human_proc = None
        self.myo_proc = None
        self.urdt_proc = None
        self.gripper_proc = None
        self.rosbag_proc = None

        self.robotMove_clicked_flag = False

        self.user_path = DATA_PATH

        # New user dialog initialization
        self.Dialog = QDialog()
        self.NewUserTool = NewUser(self)


    def open_new_user_dialog(self):
        self.NewUserTool.setupUi(self.Dialog)

        # call randomIDcreator script here
        self.NewUserTool.createButton.clicked.connect(lambda: self.NewUserTool.create_random_ID(
                                                                self.NewUserTool.nameLineEdit.text(),
                                                                self.NewUserTool.heightLineEdit.text(),
                                                                self.NewUserTool.armLengthLineEdit.text(),
                                                                self.NewUserTool.leftHandcheckBox.isChecked()))


        # self.Dialog.closeEvent = self.custom_closeEvent
        # self.Dialog.accepted = self.custom_accepted
        # self.NewUserTool.buttonBox.accepted.connect(self.Dialog.accepted)
        # self.NewUserTool.buttonBox.rejected.connect(self.Dialog.closeEvent)
        self.NewUserTool.buttonBox.accepted.connect(self.custom_accepted)
        self.NewUserTool.buttonBox.rejected.connect(self.custom_closeEvent)

        self.Dialog.show()
        self.Dialog.exec_()

    def custom_closeEvent(self):
        self.RecordPlot.recordtextEdit.append("No user created")
        #TODO: user already exist. delete manually

    def custom_accepted(self):
        new_folder = DATA_PATH+"users/"+str(self.NewUserTool.userID)
        SELECTED_ID = self.NewUserTool.userID
        try:
            mkdir(new_folder)
            self.RecordPlot.recordtextEdit.append("New user is ready with ID number: "+str(self.NewUserTool.userID))
        except FileExistsError:
            self.RecordPlot.recordtextEdit.append("The user is already registered:  "+str(self.NewUserTool.userID))


    def select_user(self):
        chdir(DATA_PATH+"users/")
        proc = subprocess.Popen(['ls','-l'], stdout=subprocess.PIPE)
        self.RecordPlot.users_list = []
        self.RecordPlot.userComboBox.addItems([''])
        for user in str(proc.stdout.read()).split('\\n')[1:-1]:
            user_folder_info = re.split(r"\s+", user)[-4:] # ['juni', '20', '23:26', '0']
            user_info = str(user_folder_info[3]+" - "+user_folder_info[1]+user_folder_info[0]+user_folder_info[2])
            # self.RecordPlot.users_list.append(user_folder_info)
            self.RecordPlot.userComboBox.addItem(user_info)

        # Searchable combobox
        self.RecordPlot.userComboBox.setEditable(True)
        self.RecordPlot.userComboBox.completer().setCompletionMode(QCompleter.PopupCompletion) 
        self.RecordPlot.userComboBox.setInsertPolicy(QComboBox.NoInsert) 
        # for trial_no in range(self.RecordPlot.trialComboBox.count()):
        #     print(self.RecordPlot.trialComboBox.itemText(trial_no))

        # print(self.RecordPlot.userComboBox.currentText())
        self.RecordPlot.userComboBox.currentTextChanged.connect(self.user_combo_changed)
        self.RecordPlot.trialComboBox.currentTextChanged.connect(self.trial_combo_changed)
    
    def user_combo_changed(self):
        self.RecordPlot.selectedID = self.RecordPlot.userComboBox.currentText().replace('-', ' ').split()[0]
        chdir(DATA_PATH+"users/"+self.RecordPlot.selectedID)
        proc = subprocess.Popen(['ls'], stdout=subprocess.PIPE)
        trial_files = str(proc.stdout.read())[2:-1].split('\\n')
        # trial_no = re.split('(?:\sb\'|\\n)\s',str(proc.stdout.read()))
        self.trial_no = len(trial_files)-1
        # for trial_no in range(self.RecordPlot.trialComboBox.count()):
        #     print(self.RecordPlot.trialComboBox.itemText(self.trial_no))
            
        # self.RecordPlot.userComboBox.setItemText(3, "ASD")
        self.RecordPlot.trialComboBox.setEnabled(True)

    def trial_combo_changed(self):
        self.RecordPlot.trial_no = self.RecordPlot.trialComboBox.currentText()
        self.RecordPlot.recordtextEdit.append("User no:"+str(self.RecordPlot.selectedID)+" trial no: "+str(self.RecordPlot.trial_no))
        self.RecordPlot.humanCalibrateButton.setEnabled(True)


    def startRecordPlotWindow(self):
        self.RecordPlot = RecordPlotWindow(self)
        self.setCentralWidget(self.RecordPlot)
        self.setWindowTitle("HRC Training")

        self.RecordPlot.awindaButton.setEnabled(False)
        self.RecordPlot.humanCalibrateButton.setEnabled(False)
        self.RecordPlot.humanJointResetButton.setEnabled(False)
        self.RecordPlot.emgResetButton.setEnabled(False)
        self.RecordPlot.humanInitiateButton.setEnabled(True)
        self.RecordPlot.measureTresholdButton.setEnabled(False)
        self.RecordPlot.setTresholdButton.setEnabled(False)
        self.RecordPlot.gripperInitiateButton.setEnabled(False)
        self.RecordPlot.robotMoveButton.setEnabled(False)
        self.RecordPlot.trialComboBox.setEnabled(False)

        self.RecordPlot.roscoreButton.clicked.connect(self.roscore_clicked)
        self.RecordPlot.awindaButton.clicked.connect(self.awinda_clicked)
        self.RecordPlot.humanCalibrateButton.clicked.connect(self.humanCalibrate_clicked)
        self.RecordPlot.humanJointResetButton.clicked.connect(self.humanJointReset_clicked)
        self.RecordPlot.emgResetButton.clicked.connect(self.emgReset_clicked)
        self.RecordPlot.humanInitiateButton.clicked.connect(self.humanInitiate_clicked)
        self.RecordPlot.measureTresholdButton.clicked.connect(self.measureTreshold_clicked)
        self.RecordPlot.setTresholdButton.clicked.connect(self.setTreshold_clicked)
        self.RecordPlot.gripperInitiateButton.clicked.connect(self.gripperInitiate_clicked)
        self.RecordPlot.robotMoveButton.clicked.connect(self.robotMove_clicked)
        self.RecordPlot.buttonBox.clicked.connect(self.stop_all_roslaunch)

        self.RecordPlot.newUserButton.clicked.connect(self.open_new_user_dialog)
        self.RecordPlot.selectUserButton.clicked.connect(self.select_user)

        self.RecordPlot.trialComboBox.addItems([str(x) for x in range(1,11)])
        # self.RecordPlot.userComboBox.setItemData(0, QFont('Verdana'), Qt.FontRole) ## TODO maybe

        self.RecordPlot.robotForceLineEdit.setToolTip("13 - 40")
        self.RecordPlot.robotSpeedLineEdit.setToolTip("0.1 - 0.8")
        self.RecordPlot.robotApproachSpeedLineEdit.setToolTip("0.2 (slow) - 0.03 (fast)")
        
        self.RecordPlot.recordtextEdit.setText("WELCOME to HVL Robotics HRC TRAINING GAME!")
        self.RecordPlot.recordtextEdit.verticalScrollBar().setValue(self.RecordPlot.recordtextEdit.verticalScrollBar().maximum())

        self.show()


    def roscore_clicked(self):
        self.rosTimer.start(10)
        self.guiTimer.start(500)
        self.ros_node.init_subscribers_and_publishers()
        self.start_single_roslaunch('/launch/gui.launch') # I need this to set a UUID for later added nodes
        sleep(1)
        self.add_rosnode("hrc_training", "visualize_and_gamify.py", "visualize_and_gamify")

        self.RecordPlot.awindaButton.setEnabled(True)


    def awinda_clicked(self):
        self.add_rosnode("awindamonitor", "awindamonitor", "awindamonitor")
        # self.RecordPlot.humanCalibrateButton.setEnabled(True)


    def humanCalibrate_clicked(self):
        chdir(self.pkg_path)
        # proc = subprocess.Popen(['ls'], stdout=subprocess.PIPE)
        # print(str(proc.stdout.read()))
        # sys.exit()
        self.human_proc = subprocess.Popen(["sh", "sh/human.sh"]) ## this will halt the system. You will use Popen: https://stackoverflow.com/questions/16855642/execute-a-shell-script-from-python-subprocess
        sleep(1.0)
        self.myo_proc = subprocess.Popen(["sh", "sh/myo.sh"])

        self.RecordPlot.emgResetButton.setEnabled(True)
        self.RecordPlot.humanInitiateButton.setEnabled(True)
        self.RecordPlot.humanJointResetButton.setEnabled(True)
        self.RecordPlot.measureTresholdButton.setEnabled(True)

    
    def measureTreshold_clicked(self):
        self.thresholdsTimer.start(10)
        self.RecordPlot.measureTresholdButton.setEnabled(False)
        self.RecordPlot.setTresholdButton.setEnabled(True)
        self.RecordPlot.rightElbowCurrentLineEdit.setEnabled(True)
        self.RecordPlot.leftElbowCurrentLineEdit.setEnabled(True)
        self.RecordPlot.emgCurrentLineEdit.setEnabled(True)
        self.RecordPlot.elbowTresholdLineEdit.setEnabled(False)
        self.RecordPlot.emgTresholdLineEdit.setEnabled(False)


    def setTreshold_clicked(self):
        self.thresholdsTimer.stop()
        self.RecordPlot.measureTresholdButton.setEnabled(True)
        self.RecordPlot.setTresholdButton.setEnabled(False)
        self.RecordPlot.rightElbowCurrentLineEdit.setEnabled(False)
        self.RecordPlot.leftElbowCurrentLineEdit.setEnabled(False)
        self.RecordPlot.emgCurrentLineEdit.setEnabled(False)
        self.RecordPlot.elbowTresholdLineEdit.setEnabled(True)
        self.RecordPlot.emgTresholdLineEdit.setEnabled(True)



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
        # subprocess.Popen(["rosnode", "kill", "/myo_raw", "/emg_to_gripper", "/world_to_myo_tf_publisher"])
        subprocess.Popen(["rosnode", "kill", "/myo_raw", "/emg_sum_node", "/world_to_myo_tf_publisher"])
        sleep(2.0)
        chdir(self.pkg_path)
        subprocess.Popen(["sh", "sh/myo.sh"])
        self.RecordPlot.recordtextEdit.append("EMG reset DONE")
        self.RecordPlot.emgResetButton.setEnabled(True)
        # TODO: check if EMG sum is changing over time



    def humanInitiate_clicked(self):
        self.add_rosnode("arm_motion_controller_py3", "wrist_to_robot_2arms.py", "wrist_to_robot_2arms")
        self.add_rosnode("imu_human_pkg", "action_server.py", "action_server")
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

        sleep(1)
        self.RecordPlot.recordtextEdit.append(self.RecordPlot.elbowTresholdLineEdit.text())
        self.RecordPlot.recordtextEdit.append(str(len(self.RecordPlot.elbowTresholdLineEdit.text())))
        self.RecordPlot.recordtextEdit.append(self.RecordPlot.emgTresholdLineEdit.text())
        self.RecordPlot.recordtextEdit.append(str(len(self.RecordPlot.emgTresholdLineEdit.text())))
        self.RecordPlot.recordtextEdit.verticalScrollBar().setValue(self.RecordPlot.recordtextEdit.verticalScrollBar().maximum())
        self.RecordPlot.recordtextEdit.repaint()

        # TODO: set elbow_height_th, emg_sum, hand_dominance parameters
        if(len(self.RecordPlot.elbowTresholdLineEdit.text())>0 and len(self.RecordPlot.emgTresholdLineEdit.text())>0):
            colift_force = int(self.RecordPlot.robotForceLineEdit.text())
            release_speed = float(self.RecordPlot.robotSpeedLineEdit.text())
            approach_speed = float(self.RecordPlot.robotApproachSpeedLineEdit.text())
            elbow_th = float(self.RecordPlot.elbowTresholdLineEdit.text())
            emg_th = int(self.RecordPlot.emgTresholdLineEdit.text())
            self.ros_node.set_params(elbow_height_th=elbow_th, emg_sum_th=emg_th, user_id=self.RecordPlot.selectedID, trial_no=self.RecordPlot.trial_no, approach_speed=approach_speed, colift_force=colift_force, release_speed=release_speed)

            self.RecordPlot.gripperInitiateButton.setEnabled(True)
        else:
            print("Elbow and EMG calibrations are not completed")
            self.RecordPlot.recordtextEdit.append("Elbow and EMG calibrations are not completed")


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
        if not self.robotMove_clicked_flag:
            # self.urdt_proc = subprocess.Popen(["/home/gizem/venv/venv-ur/bin/python3.8", "/home/gizem/catkin_ws/src/arm_motion_controller_py3/src/robot_move_node.py"])  ## For other PCs or multiple PCs this needs to be changes. Not modular.
            self.urdt_proc = subprocess.Popen(["/home/gizem/venv/venv-ur/bin/python3.8", "/home/gizem/catkin_ws/src/imu_human_pkg/imu_human_pkg/src/hrc_state_machine_restartable.py"])

            while not rospy.get_param('/robot_move_started'):
                print("waiting for robot")
                rospy.sleep(0.5)
                if rospy.is_shutdown():
                    sys.exit() 
            
            print("robot started")

            chdir(DATA_PATH+'users/'+str(self.RecordPlot.selectedID))
            print(getcwd())
            if not self.RecordPlot.loggingcheckBox.isChecked():
                now_str = str(self.ros_node.now.data).replace(' ', '_').replace('.','_').replace(':', '_').replace('-', '_')
                print(now_str, "rosbag starting")
                subprocess.Popen(["rosparam", "dump", now_str+'-'+str(self.RecordPlot.selectedID)+'-'+str(self.RecordPlot.trial_no)+'.yaml'])
                self.rosbag_proc = subprocess.Popen(["rosbag", "record", "-a", "-O", now_str+"_"+str(self.RecordPlot.selectedID)+"_"+self.RecordPlot.trial_no])

            self.RecordPlot.robotMoveButton.setText("Robot Stop")
            self.logging_started_flag = True
            self.ros_node.start_time = time.time()
        else:
            print("before buttons:", self.ros_node.registered_buttons)
            print("before score:", self.ros_node.score_val)
            self.ros_node.reset()
            self.ros_node.update()
            print("after buttons:", self.ros_node.registered_buttons)
            print("after score:", self.ros_node.score_val)
            try:
                self.urdt_proc.send_signal(signal.SIGINT)
                self.rosbag_proc.send_signal(signal.SIGINT)
                print("killed processes")
                del self.urdt_proc
                del self.rosbag_proc
            except Exception as e: 
                logging.exception("An exception was thrown!")

            # subprocess.Popen(["rosnode", "kill", "/visualize_and_gamify"])
            if rospy.has_param('/robot_move_started'):
                rospy.set_param('/robot_move_started', False)
            if rospy.has_param('/colift_set'):
                rospy.set_param('/colift_set', False)
            if rospy.has_param('/elbow_height_th'):
                rospy.delete_param('/elbow_height_th')
            if rospy.has_param('/emg_sum_th'):
                rospy.delete_param('/emg_sum_th')
            self.RecordPlot.robotMoveButton.setText("Robot Move")

        self.robotMove_clicked_flag = not self.robotMove_clicked_flag


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
            # self.human_proc.kill()
            # self.myo_proc.kill()
            # self.urdt_proc.kill()
            # self.gripper_proc.kill()
            # self.rosbag_proc.kill()

            self.human_proc.send_signal(signal.SIGINT)
            self.myo_proc.send_signal(signal.SIGINT)
            self.urdt_proc.send_signal(signal.SIGINT)
            self.gripper_proc.send_signal(signal.SIGINT)
            self.rosbag_proc.send_signal(signal.SIGINT)
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

    def measure_thresholds_update(self):
        self.RecordPlot.leftElbowCurrentLineEdit.setText("%.3f"%self.ros_node.left_elbow_current)
        self.RecordPlot.rightElbowCurrentLineEdit.setText("%.3f"%self.ros_node.right_elbow_current)
        self.RecordPlot.emgCurrentLineEdit.setText("%4.0f"%self.ros_node.emg_sum)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    """LOG: python3 -u training.py | tee >(ts "%d-%m-%y %H_%M_%S" > /home/gizem/Insync/giat@hvl.no/Onedrive/HVL/Human_Experiments/data/terminal_logs/log_file.log)"""
    print("System started")
    # app.setStyleSheet(stylesheet)
    w = MainWindow()
    sys.exit(app.exec_())