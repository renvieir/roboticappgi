#!/usr/bin/env python


from control_model import ControlModel
from sordalane import *
from path_following import PathFollowing
from math import *

if __name__ == '__main__':
    try:
        #ControlModel(1, 1).run()

        points = [  (3,0,0),(6,0,0),(9,0,0),(12,0,0),
                    (13,1.5,pi/2), (13,5,pi/2), (13,9,pi/2), (13,9,-pi/2), (13,5,-pi/2), (13,1.5,-pi/2),
                    (12,0,pi),(9,0,pi),(6,0,pi),(3,0,pi), (0,0,pi)]
        # points = [ (1.2,1.2,0)]

        s = PathFollowing(points)
        s.run()

    except rospy.ROSInterruptException:
        pass

