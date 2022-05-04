#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import sys, subprocess
import time

# cmd = "roslaunch "+"/home/gizem/catkin_ws/src/hrc_training/launch/gui.launch"
# proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
# str_out = str(proc.stdout.read())
# print(str_out)


# def flush_then_wait():
#     sys.stdout.flush()
#     sys.stderr.flush()
#     time.sleep(0.5)


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)

        # sys.stdout.write("Script stdout 1\n")
        # sys.stdout.write("Script stdout 2\n")
        # sys.stdout.write("Script stdout 3\n")
        # sys.stderr.write("Total time: 00:05:00\n")
        # sys.stderr.write("Total complete: 10%\n")
        # flush_then_wait()


        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    # p_kill = subprocess.Popen(["pkill", "-9", "ros"])