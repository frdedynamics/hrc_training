#!/bin/sh
input_suffix=".ui"
output_suffix=".py"

input_file=$1$input_suffix
output_file=$1$output_suffix


pyuic5 -x /home/gizem/catkin_ws/src/hrc_training/ui/$input_file -o /home/gizem/catkin_ws/src/hrc_training/src/Classes/$output_file
