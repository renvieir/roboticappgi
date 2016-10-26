#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Algoritmo derivado baseado no livro Probabilistic Robotics, pg 284
    Referencias Numpy
    http://cs231n.github.io/python-numpy-tutorial/
    https://docs.scipy.org/doc/numpy-dev/user/quickstart.html
    http://docs.scipy.org/doc/numpy/reference/arrays.nditer.html
    https://docs.python.org/2/library/math.html
"""

import numpy as np
import matplotlib.pyplot as plt
import math 
from bresenham import Bresenham
from sensor_msgs.msg import LaserScan
from scipy.misc import toimage


class Cartesian:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class OccupancyGridMap:
    def __init__(self, m=500, n=500, apriori=0.5):
        """ 
            inicializa um array de tamanho fixo (m,n) com valores apriori para ser o mapa
            m e n sao as dimensoes do mapa
            apriori eh o valor da probabilidade inicial
        """
        self._m = m
        self._n = n
        self._apriori = apriori
        self._occ = 0.99
        self._free = 1 - self._occ

        # self.occupancy_map = np.full((m, n), apriori)
        self.occupancy_map = [[[0.5] for _ in range(0, m)] for _ in range(0, n)]

        self.position_in_map = Cartesian(m/2, n/2)
        self.granularity = 10.0/m # 1cm in meters (granularidade)

    def set_occ_map_cell(self, i, j):
        self.occupancy_map[i][j] = [self._occ]

    def set_free_map_cell(self, i, j):
        self.occupancy_map[i][j] = [self._free]

    def map_cell_add_value(self, i, j, value):
        i, j = self.validate_indexes(i, j)           
        self.occupancy_map[i][j].append(value)

    def validate_indexes(self, i, j):
        if i>=self._m:
            i = self._m -1
        if j>=self._n:
            j = self._n -1    
        if i < 0:
            i=0
        if j<0:
            j=0
        return i, j

    def print_map(self):
        ocmap = []
        for line in self.occupancy_map:
            ocmap.append([cell[0] for cell in line])

        npmap = np.array(ocmap)
        toimage(npmap, cmin=0.0, cmax=1.0).save('occupancy_map.jpeg')

    def update_map(self, pose, laser_scan):
        # pinta a posicao do robo no mapa
        self.set_occ_map_cell(self.position_in_map.x, self.position_in_map.y)

        # pega as leituras, coloca na sua posição no mapa e seta o caminho como livre
        self.process_data_to_map(laser_scan, pose)

        # self.print_map()
        # 3. percorre a matriz e calcula as probabilidades
        for i in range(0, self._m):
            for j in range(0, self._n):
                l = self.occupancy_map[i][j]
                self.occupancy_map[i][j] = [np.mean(l)]

        self.print_map()
        print 'Updating map...'

    def process_data_to_map(self, laser_scan, pose):
        """
        Process laser readings and transform it to positions on grid map
        :param laser_scan: data read from Laser
        :param pose: Robot pose
        :return:
        """
        origin = laser_scan.angle_min
        increment = laser_scan.angle_increment

        pose_x = pose[0]
        pose_y = pose[1]
        pose_theta = pose[2]

        for num, range_ in enumerate(laser_scan.ranges):

            # convert this laser reading to polar coordinate
            distance=range_ 
            angle=origin + num*increment

            # get world coordinate of this laser reading
            x=math.cos(angle+pose_theta)*distance
            y=math.sin(angle+pose_theta)*distance
                
            # and transform it to matrix index
            i = int(self.position_in_map.x + x/self.granularity)
            j = int(self.position_in_map.y + y/self.granularity)

            # get path from robot position to each point on laser
            path = Bresenham((self.position_in_map.x, self.position_in_map.y),
                             (i, j)).path

            # set each point on path to free
            for p in path:
                self.map_cell_add_value(p[0], p[1], self._free)

            # set the position for this laser reading as occupied
            self.map_cell_add_value(i,j,self._occ)


if __name__ == '__main__':
    from robots import Robot

    robot = Robot()
    ogm = OccupancyGridMap()
    robot.start(ogm.update_map)
