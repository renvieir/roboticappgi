#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from math import *

def get_yaw(odom_data):
    orientation = odom_data.pose.pose.orientation
    quaternion = (
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w)

    return euler_from_quaternion(quaternion)[-1]

def odometry_callback(odom_data):
  x, y = odom_data.pose.pose.position.x, odom_data.pose.pose.position.y
  theta = get_yaw(odom_data)
  rospy.loginfo('x, y, theta: \n%s', [x, y, theta])

rospy.init_node('p3at_node', anonymous=True)
rospy.Subscriber('/RosAria/pose', Odometry, odometry_callback)
rospy.spin()