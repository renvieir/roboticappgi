#!/usr/bin/env python
# -*- coding: utf-8 -*-

from control_model import *


class Sordalane(ControlModel):

    def func_sinc(self, theta):
        sinc = 1.0;
        if abs(theta) > 0.001:
            sin(theta)/theta;
        return sinc;

    def theta_d(self, x, y):
        if x==0.0:
            x = 0.1;
        return 2.0*atan(y/x);

    def func_sign(self, x, y):
        if ((x==0.0 and y<0.0) or x>0.0):
            return 1.0;
        return -1.0;

    def coord_circular(self, x, y):
        return self.func_sign(x,y)*sqrt(x*x + y*y)/(self.func_sinc(self.theta_d(x,y)/2.0));

    def norm_rad(self, angle):
        while angle > pi :
            angle -= 2.0*pi;
        while angle < -pi :
            angle += 2.0*pi;
        return angle;

    def funcb1(self, x, y, theta, alpha):
        try:
            mybeta = y/x;
        except ZeroDivisionError:
            mybeta = 0

        thetad = self.theta_d(x,y);
        if thetad != 0:
            return cos(theta)*((thetad/mybeta) - 1.0) \
                    + sin(theta)*(thetad/2.0*(1.0-1.0/pow(mybeta,2.0)) \
                    + 1.0/mybeta);
        return cos(alpha);    

    def funcb2(self, x, y, theta):
        try:
            mybeta = y/x;
        except ZeroDivisionError:
            mybeta = 0

        thetad = self.theta_d(x,y);
        if thetad != 0:
            return cos(theta)*(2.0*mybeta/((1.0+pow(mybeta,2.0))*x)) - sin(theta)*(2.0/((1.0+pow(mybeta,2.0))*x));
        return sin(thetad/2.0 - theta) * (2.0 / self.func_sinc(thetad/2.0));

    def gama2(self, gama, a):
        return gama/(1.0 + abs(a));

    def refresh_position(self):
        gama1 = 1.0
        k = 1.0

        self.err_x = self.x - self.x_p;
        self.err_y = self.y - self.y_p;
        theta_m = self.theta - self.theta_p;

        x_m = (cos(self.theta_p)*self.err_x + sin(self.theta_p)*self.err_y);
        y_m = (-sin(self.theta_p)*self.err_x + cos(self.theta_p)*self.err_y);            

        a = self.coord_circular(x_m,y_m);   
        alpha = self.norm_rad(theta_m-self.theta_d(x_m,y_m));

        fb1 = self.funcb1(x_m,y_m,theta_m, alpha);
        self.u = -self.gama2(gama1,a)*fb1*a;

        fb2 = self.funcb2(x_m,y_m,theta_m);
        self.w = (-fb2*self.u) - (k*alpha);