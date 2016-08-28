#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from math import *


class Robot:
  """
    This class represents a robot that subscribe for laser and odom topics 
    and expect a function to handles this data
  """
  def __init__(self, robot_name='p3at', odom_topic='/RosAria/pose', laser_topic='/scan'):
    self.name = robot_name
    self.odom_topic = odom_topic
    self.laser_topic = laser_topic
    self.odom_data = (0, 0, 0)
    self.laser_data = []

  def get_yaw(self, odom_data):
      orientation = odom_data.pose.pose.orientation
      quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)

      return euler_from_quaternion(quaternion)[-1]

  def odometry_callback(self, odom_data):
    x, y = odom_data.pose.pose.position.x, odom_data.pose.pose.position.y
    theta = self.get_yaw(odom_data)
    self.odom_data = (x, y, theta)

  def laser_callback(self, laser_data):
    self.laser_data = laser_data

  def make_subscriptions(self):
    rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback)
    rospy.Subscriber(self.laser_topic, LaserScan, self.laser_callback)

  def start(self, main_callback):
    rospy.init_node(self.name+'_node', anonymous=True)
    self.make_subscriptions()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
      rate.sleep()
      main_callback(self.odom_data, self.laser_data)


def print_odom_data(odom_data):
  print odom_data

def print_laser_data(laser_data):
  print laser_data.ranges[0:3]

  print '----------------'
  print laser_data.angle_min
  print laser_data.angle_max
  print laser_data.angle_increment
  print laser_data.range_min
  print laser_data.range_max
  print len(laser_data.ranges) 
 print '----------------'

def prints(odom_data, laser_data):
  print_odom_data(odom_data)
  print_laser_data(laser_data)


if __name__ == '__main__':
  r = Robot()
  r.start(prints)