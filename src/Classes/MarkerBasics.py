#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
import numpy
from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState
import math

class MarkerBasics(object):
    def __init__(self, topic_id):
        marker_topic = topic_id+'marker'
        self.marker_objectlisher = rospy.Publisher(marker_topic, Marker, queue_size=1)
        self.rate = rospy.Rate(25)
        self.init_marker(index=0)


    def init_marker(self, index=0):
        self.marker_object = Marker()
        self.change_frame(frame="base_link", index=0)
        self.marker_object.type = Marker.ARROW
        self.marker_object.action = Marker.ADD
        self.marker_object.scale.x = 0.4
        self.marker_object.scale.y = 0.05
        # self.marker_object.scale.z = 0.294151609476
        self.marker_object.scale.z = 0.05

        self.marker_object.color.r = 0
        self.marker_object.color.g = 255
        self.marker_object.color.b = 0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0


        # self.change_position(x=0.0, y=0.0, z=0.0)
        # self.change_orientation(pitch=0.0, yaw=0.0)
        # self.change_scale()
        # self.change_colour(R=1.0, G=0.0, B=0.0)

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)


    def change_orientation(self, pitch, yaw):
        """
        Roll doesnt make any sense in an arrow
        :param pitch: Up Down. We clip it to values [-1.5708,1.5708]
        :param yaw: Left Right , No clamp
        :return:
        """
        pitch = numpy.clip(pitch, -1.5708,1.5708)

        q = tf.transformations.quaternion_from_euler(0.0, pitch, yaw)

        self.marker_object.pose.orientation.x = q[0]
        self.marker_object.pose.orientation.y = q[1]
        self.marker_object.pose.orientation.z = q[2]
        self.marker_object.pose.orientation.w = q[3]


    def change_position(self, x, y, z):
        """
        Position of the starting end of the arrow
        :param x:
        :param y:
        :param z:
        :return:
        """

        my_point = Point()
        my_point.x = x
        my_point.y = y
        my_point.z = z
        self.marker_object.pose.position = my_point
        #rospy.loginfo("PositionMarker-X="+str(self.marker_object.pose.position.x))


    def change_colour(self, R, G, B):
        """
        All colours go from [0.0,1.0].
        :param R:
        :param G:
        :param B:
        :return:
        """

        self.marker_object.color.r = R
        self.marker_object.color.g = G
        self.marker_object.color.b = B
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0


    def change_scale(self, s_x=1.0, s_y=0.1, s_z=0.1):
        """
        :param s_x:
        :param s_y:
        :param s_z:
        :return:
        """

        self.marker_object.scale.x = s_x
        self.marker_object.scale.y = s_y
        self.marker_object.scale.z = s_z


    def change_frame(self, frame, ns="human", index=0):
        """
        :param frame:
        :return:
        """
        self.marker_object.header.frame_id = frame
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = ns
        self.marker_object.id = index


    def update_marker(self, frame, ns, index, position, orientation, pressure, min_pressure=0.0, max_pressure=10.0):
        """
        :param position: [X,Y,Z] in the world frame
        :param pressure: Magnitude
        :param orientation: [Pitch,Yaw]
        :return:
        """
        #self.change_frame(frame=frame, ns=ns, index=index)
        self.change_position(x=position[0], y=position[1], z=position[2])
        self.change_orientation(pitch=orientation[0], yaw=orientation[1])
        self.change_scale(s_x = pressure)

        R,G,B = self.pressure_to_wavelength_to_rgb(pressure=pressure,
                                                   min_pressure=min_pressure,
                                                   max_pressure=max_pressure,
                                                   gamma=0.8)

        rospy.logdebug("R,G,B=["+str(R)+", "+str(G)+", "+str(B)+"]")

        self.change_colour(R=R, G=G, B=B)

        self.marker_objectlisher.publish(self.marker_object)