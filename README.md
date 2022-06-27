# HRC Training for user tests

This repository contains the bridging code between `imu_human_pkg`, `arm_motion_pkg_py3`(will be depreciated) `data_logger`(now publicly available - will be depreciated) and automizes the whole system. 

## Dependencies:
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
