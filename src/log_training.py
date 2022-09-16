#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys, os 
import training
from datetime import datetime

orig_stdout = sys.stdout  # capture original state of stdout

te = open('log.txt','w')  # File where you need to keep the logs

class Unbuffered:
    def __init__(self, stream):
        self.stream = stream

    def write(self, data):
        self.stream.write(data)
        self.stream.flush()
        te.write(data.replace("\n", " [%s]\n" % str(datetime.now())))    # Write the data of stdout here to a text file as well

    def flush(self):
        pass

sys.stdout=Unbuffered(sys.stdout)




#######################################
##  Feel free to use print function  ##
#######################################

print("Logging started =)")
# training.log_main()

#######################################
##  Feel free to use print function  ##
#######################################




# Stop capturing printouts of the application from Windows CMD
sys.stdout = orig_stdout  # put back the original state of stdout
te.flush()  # forces python to write to file
te.close()  # closes the log file