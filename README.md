# HRC Training for user tests

This repository contains the bridging code between `imu_human_pkg`, `arm_motion_pkg_py3`(will be depreciated) `data_logger`(now publicly available - will be depreciated) and automize the whole system. It has its own data logger.

## Dependencies:
- `imu_human_pkg`
- `arm_motion_pkg_py3`
- `ur_rtde`(Python)
- `ros_magic_pkg`
- `ros_robotiq_urcap_control` (in `imu_human_pkg>additional` folder)

## How to run:
- Start `roscore`.
- `roscd hrc_training/src`
- `./training.py`
