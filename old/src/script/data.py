#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The files should be layouted as following:
    - All generated CSV files should be placed somewhere within the 'Output' folder
    - Every set of CSV files that belong to one Tester should be placed in a separate subfolder
    for example, if a user Ahmet ran the simulation 10 times, then 10 CSV files were generated,
    we place all of them in a folder called 'Ahmet' or some random number, it doesn't really matter.
    - Basically every subfolder represents one set of training runs. One person yani
    - The generated CSV files should be named with their timestamped 
    so that there filenames are can be sorted in increasing order.
#todo: deal with different simulation scenarios (with vs without axis alignment)
"""

import os
from datetime import datetime



OUTPUT_FOLDER = "Output/caghan/finger"
OUTPUT_FILENAME_PREFIX = "finger"

ID_MARK = 'mark'
ID_PEDAL = 'pedal'
ID_ANGLE_X = 'angle_x'
ID_ANGLE_Y = 'angle_y'
ID_ANGLE_Z = 'angle_z'
ID_DISTANCE = 'distance'
ID_TIME = 'elapsed time'

DATA_LABELS = (ID_MARK, ID_PEDAL, ID_ANGLE_X, ID_ANGLE_Y, ID_ANGLE_Z, ID_DISTANCE, ID_TIME)
DATA_INDICES = {
        ID_MARK: 0,
        ID_PEDAL: 1,
        ID_ANGLE_X: 2,
        ID_ANGLE_Y: 3,
        ID_ANGLE_Z: 4,
        ID_DISTANCE: 5,
        ID_TIME: 6
        }



def get_new_filename():
    postfix = datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + ".csv"
    filename = OUTPUT_FILENAME_PREFIX + "_" + postfix
    # if folder doesn't exist, create it
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
    filename = os.path.join(OUTPUT_FOLDER, filename)
    return filename
