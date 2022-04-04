#!/bin/sh
input_prefix="main"
input_suffix=".ui"
output_prefix="main"
output_suffix=".py"

#input_file=$input_prefix$1$input_suffix
input_file=$input_prefix$input_suffix
output_file=$output_prefix$1$output_suffix


pyuic5 -x /home/gizem/catkin_ws/src/hrc_training/ui/$input_file -o /home/gizem/catkin_ws/src/hrc_training/src/Classes/$output_file
