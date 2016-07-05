#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sordalane import *


class PathFollowing(Sordalane):

    def __init__(self, points):
        Sordalane.__init__(self,0,0,0)
        self.points = points

    def run(self):
        i = 0
        while not rospy.is_shutdown():
            self.refresh_position()
            self.publish_velocities()

            if len(self.points) < i and (self.err_x < 0.2 or self.err_y < 0.2):
                i+=1
                (self.x_p, self.y_p, self.theta_p) = self.points[i]