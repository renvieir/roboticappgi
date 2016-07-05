#!/usr/bin/env python


from control_model import ControlModel
from sordalane import *
from path_following import PathFollowing
from math import *

if __name__ == '__main__':
    try:
        # ControlModel(1, 1).run()

        points = [ (1,1,pi/2), (2,2,-pi), (-1,1,-pi/2), (0,0,0)]

        s = PathFollowing(points)
        s.run()


    except rospy.ROSInterruptException:
        pass

