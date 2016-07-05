#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sordalane import *


class PathFollowing(Sordalane):

    def __init__(self, points):
        Sordalane.__init__(self,0,0,0)
    	print type(points)
        self.points = points

    def run(self):
        i = 0
        print i, len(self.points)
        (self.x_p, self.y_p, self.theta_p) = self.points[i]        
        while not rospy.is_shutdown():
            self.refresh_position()
            self.publish_velocities()

            if len(self.points) > i and (fabs(self.err_x) < 0.2 and fabs(self.err_y) < 0.2):
                i+=1
                print self.points[i], self.err_x, self.err_y
                (self.x_p, self.y_p, self.theta_p) = self.points[i]
