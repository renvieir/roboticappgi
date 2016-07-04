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

class ControlModel:


  def __init__(self):
    self.x = 0
    self.y = 0
    self.theta = 0
    self.x_p = -1
    self.y_p = 0
    self.pub = None


  def callback(self, odom_data):
      self.x, self.y = odom_data.pose.pose.position.x, odom_data.pose.pose.position.y
      self.theta = get_yaw(odom_data)


  def run(self):
    rospy.init_node('meunode', anonymous=True)
    rospy.Subscriber('/RosAria/pose', Odometry, self.callback)
    self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
      self.refresh_position()


  def refresh_position(self):
    err_x = self.x_p - self.x;
    err_y = self.y_p - self.y;

    rospy.loginfo('Erro de Posicao \n%s', [err_x, err_y])

    x_m = (cos(self.theta)*err_x + sin(self.theta)*err_y);
    y_m = (-sin(self.theta)*err_x + cos(self.theta)*err_y);      

    rospy.loginfo('xM, yM, teta: \n%s', [x_m, y_m, self.theta])

    # desconto da frente do robô
    x_m = x_m - 0.25;
    
    k1 = 0.5 /(1 + fabs(x_m));
    k2 = (70.0)*(pi/180)/(1 + fabs(y_m));

    u = k1*x_m;
    w = k2*y_m;

    rospy.loginfo('u, w: \n%s', [u, w])

    vel_msg = Twist()

    # velocidades lineares
    vel_msg.linear.x = u; # seta a velocidade liner no eixo x para a frente do robo
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    
    # velocidades angulares
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = w; #seta o valor de rotação do robo p3at

    self.pub.publish(vel_msg)


if __name__ == '__main__':
  try:
    ControlModel().run()
  except rospy.ROSInterruptException:
    pass

