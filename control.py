#!/usr/bin/env python


from control_model import ControlModel
from sordalane import *
from path_following import PathFollowing
from math import *

if __name__ == '__main__':
    try:
        #ControlModel(1, 1).run()

        points = [  (11,0,0),(12.5,1.5,pi/2), (12.5,9,pi/2), (12.5,1.5,-pi/2),(11,0,pi),(0,0,pi)]

        s = PathFollowing(points)
        s.run()

    except rospy.ROSInterruptException:
        pass

