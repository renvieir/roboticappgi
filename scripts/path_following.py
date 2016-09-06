#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sordalane import *


class PathFollowing(Sordalane):

    def __init__(self, points):
        Sordalane.__init__(self,0,0,0)
    	print type(points)
        self.points = points
        self.u_max = 0.4
        self.raio = 0.2

    def run(self):

        last = self.points[-1]

        for index, p in enumerate(self.points):
            (self.x_p, self.y_p, self.theta_p) = p

            overload_velocity = True
            if p is last:
                overload_velocity = False

            while not rospy.is_shutdown():
                self.refresh_position()

                # if self.u_max < self.u:
                #     self.u_max = self.u

                if overload_velocity:
                    self.u = self.u_max

                self.publish_velocities()

                if fabs(self.err_x) < self.raio and fabs(self.err_y) < self.raio:
                    print 'end point'
                    break

        self.stop()