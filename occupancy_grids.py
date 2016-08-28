"""
    Algoritmo derivado baseado no livro Probabilistic Robotics, pg 284
    Referencias Numpy
    http://cs231n.github.io/python-numpy-tutorial/
    https://docs.scipy.org/doc/numpy-dev/user/quickstart.html
    http://docs.scipy.org/doc/numpy/reference/arrays.nditer.html
    https://docs.python.org/2/library/math.html
"""

import numpy as np
import math 


class OccupancyGridMap:
    def __init__(self, m=100, n=100, apriori=0.5, angle_min=-2.35619449615, angle_max=2.35619449615):
        """ 
            inicializa um array de tamanho fixo (m,n) com valores apriori para ser o mapa
            m e n sao as dimensoes do mapa
            apriori eh o valor da probabilidade inicial
        """
        self._m = m
        self._n = n
        self._apriori = apriori
        self._occ = 0.99
        self._free = 0.01
        self._angle_min = angle_min
        self._angle_max = angle_max

        self.occupancy_map = np.full((m, n), apriori)

    def set_occ_map_cell(self, i, j):
        self.occupancy_map[i,j] = self._occ

    def set_free_map_cell(self, i, j):
        self.occupancy_map[i,j] = self._free

    def update_map(self, pose, measurement):
        map_iter = np.nditer(self.occupancy_map, op_flags=['readwrite'], flags=['f_index'])
        while not map_iter.finished:
            if in_sensor_perceptual_field(map_iter):
                # atualiza valor
                map_iter[0] = inverse_range_sensor_model(map_iter, pose, measurement)
            map_iter.iternext()

    def inverse_range_sensor_model(self, map_cell, pose, measurement):
        """
        Map_cell is a numpy.nditer element
        Pose is a (x, y, theta) tuple
        Measurement is a laser measure
        """
        'alpha=granularidade, ou tamanho que cada celula do mapa representa no mundo real'
        alpha = 0.01 #1cm
        'beta=tamanho em angulo de cada beam'
        beta = 28.6479 # 28.6479rad = 5graus

        x_i, y_i, = self.get_mass_center()'centro de massa das medidas em uma celula'
        x, y, theta = pose
        r = math.sqrt( (x_i -x)**2 + (y_i - y)**2 )
        # arco tangente retorna o angulo do centro de massa, phi eh o resultado da
        # diferenca entre o angulo de ponto de massa e o angulo do robo
        phi = math.atan2(y_i - y, x_i - x) - theta 
 
        #theta_sense eh 'o conjunto de feixes que atinge o ponto representado pela celula'
        theta_sens = self.get_theta_sens()
        # dentro do angulo phi existem varios feixes, k eh o indice do feixe mais proximo do ponto de centro de massa
        k = np.argmin(phi - theta_sens) # essa subtracao tem que ser feita element wise

        z_max = 10 # maior medida considerada no mapa
        z_k_t = 'MEDIDA_CORRESPONDENTE_A_CELULA_DO_MAPA'
        theta_k_sens = 'ANGULO_DA_MEDIDA'

        if (r > min(z_mak, z_k_t + (alpha/2))) or ((phi -theta_k_sens) > beta/2):
            return l_o
        
        if (z_k_t < z_max) and (math.fabs(r - z_k_t) < alpha/2):
            return l_occ
        
        if r <= z_k_t:
            return l_free

        raise Exception('No probability calculated')

    def in_sensor_perceptual_field(self, nditer_index):
        theta = math.atan(j/i)
        return self.angle_min < theta < self.angle_max

    def get_mass_center(self):
        return 0, 0

    def get_theta_sens(self, pose, measurement):
        # pegar
        return []

if __name__ == '__main__':
    from listener import Robot

    robot = Robot()
    ogm = OccupancyGridMap()
    robot.start(ogm.update_map)