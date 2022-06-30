# Gamified HRC Training

This repository contains a bridging code between `imu_human_pkg` (new Human Commander and Robot Commander), `arm_motion_pkg_py3`(old Human Commander - will be depreciated) `data_logger`(beta version - use `rosbag` for generic data logging for now) and automizes the whole system. The repository contains the GUI and game score node. This README file contains what is available in the package currently (30/06/2022) and new features are still being developed. The README update might be slower.

![](/home/gizem/catkin_ws/src/hrc_training/fig/hrc_training_gui.png)

### Record Groupbox:

It allows starting the Human Commander, Robot Commander and Task Environment measurements through GUI. Either new user can be created or a created user can be selected. (Note: For privacy reasons, the recorded data is automatically located outside of the repository.)

* **roscore**: Starts the ROS launch file for GUI nodes.

* **awindamonitor**: Starts the Xsens Awinda Base Station ROS node (Source code credit for:  https://github.com/Raffa87/xsense-awinda.git)

* **Select User** and **New User**: (Optional) The GUI allows recording data automatically but it is still in beta mode. Trial no is also related to user data recording.

* **Human Calibrate**: Calibrates IMUs on the body. The human should stay in N-pose. This calibration can be reseted by **Human Joint Reset** button. Similarly **EMG Reset** button resets the EMG sensor node.

* **Measure Threshold**: Starts active measurements from both elbow heights for COLIFT direction threshold and EMG for gripper activation threshold.

* **Set Thresholds**: Locks the *left elbow current*, *right elbow current*, and  *EMG threshold* line-edits so that the decided thresholds can be manually enteres in *elbow threshold* and *emg threshold* parts.

* **Human Initiate**: The initial position of the human. Also defined as the origin of human-to-robot relative mapping. Not restricted to provide "self expression" game element but suggested to have elbows bent, two palms looking downwards as follows:

  ![](/home/gizem/catkin_ws/src/hrc_training/fig/human_init.png)

* **Gripper Initiate**: Starts required TCP connections with Robotiq 2-finger adaptive gripper.

* **Robot Move**: Starts the HRC scenario in IDLE state.

### Plot Groupbox:

It is aimed to see the users learning curve, comparative user performance, comparative trial success etc. plots via GUI easily. The implementation is not completed yet. 

## Dependencies:

The packages used in the example setup are given below. However, if you have other human commander or robot commander, you need to enable them manually.

- `imu_human_pkg` 
- `arm_motion_pkg_py3`
- `ur_rtde`(Python) and a UR5e robot
- `ros_magic_pkg`
- `ros_robotiq_urcap_control` (in `imu_human_pkg>additional` folder)

## How to run:
- Start `roscore`.
- `roscd hrc_training/src`
- `./training.py`


## Troubleshoot:
- For plug-and-play usage, please plug the EMG sensor dongle first and connect the receiver Arduino after. By doing that, you make sure the **Human Initiate** button to get EMG signals from `/dev/ttyACM0` and Arduino signals from `/dev/ttyACM1`. You can change it manually by modifying the human_pkg*/launch/human.launch and set the EMG sensor port as `ttyACM1` and starting the Arduino manually with the following command: `rosrun rosserial_python serial_node.py /dev/ttyACM0`
