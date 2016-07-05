#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from math import *


class ControlModel:

  def __init__(self, xp, yp, thetap=None):
    self.x = 0
    self.y = 0
    self.theta = 0

    self.err_x = 0
    self.err_y = 0

    self.x_p = xp
    self.y_p = yp
    self.theta_p = thetap
    
    self.u = None
    self.w = None

    rospy.init_node('p3at_node', anonymous=True)
    rospy.Subscriber('/RosAria/pose', Odometry, self.odometry_callback)
    self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

  def get_yaw(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        quaternion = (
          orientation.x,
          orientation.y,
          orientation.z,
          orientation.w)

        return euler_from_quaternion(quaternion)[-1]

  def odometry_callback(self, odom_data):
      self.x, self.y = odom_data.pose.pose.position.x, odom_data.pose.pose.position.y
      self.theta = self.get_yaw(odom_data)


  def publish_velocities(self):
    vel_msg = Twist()

    # velocidades lineares
    vel_msg.linear.x = self.u; # seta a velocidade liner no eixo x para a frente do robo
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    
    # velocidades angulares
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = self.w; #seta o valor de rotação do robo p3at

    self.pub.publish(vel_msg)


  def refresh_position(self):
    self.err_x = self.x_p - self.x;
    self.err_y = self.y_p - self.y;
    # rospy.loginfo('Erro de Posicao \n%s', [err_x, err_y])

    x_m = (cos(self.theta)*self.err_x + sin(self.theta)*self.err_y);
    y_m = (-sin(self.theta)*self.err_x + cos(self.theta)*self.err_y);      
    # rospy.loginfo('xM, yM, teta: \n%s', [x_m, y_m, self.theta])

    # desconto da frente do robô
    x_m = x_m - 0.25;    
    k1 = 0.5 /(1 + fabs(x_m));
    k2 = (70.0)*(pi/180)/(1 + fabs(y_m));
    self.u = k1*x_m;
    self.w = k2*y_m;

  def run(self):
    while not rospy.is_shutdown():
      self.refresh_position()
      self.publish_velocities()