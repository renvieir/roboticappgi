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
    def __init__(self, m=500, n=500, apriori=0.5, angle_min=-2.35619449615, angle_max=2.35619449615):
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
        self._angle_min = angle_min
        self._angle_max = angle_max

        # self.occupancy_map = np.full((m, n), apriori)
        self.occupancy_map = [[[0.5] for _ in range(0, m)] for _ in range(0, n)]

        self.position_in_map = Cartesian(m/2, n/2)
        self.granularity = 10.0/m # 1cm in meters (granularidade)
        self.beta = 28.6479 # 28.6479rad = 5graus

    def set_occ_map_cell(self, i, j):
        self.occupancy_map[i][j] = [self._occ]

    def set_free_map_cell(self, i, j):
        self.occupancy_map[i][j] = [self._free]

    def map_cell_add_value(self, i, j, value):
        self.occupancy_map[i][j].append(value)

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

        for i, m in enumerate(laser_scan.ranges):

            # convert this laser reading to polar coordinate
            distance=m 
            angle=origin + i*increment

            # get world coordinate of this laser reading and transform to matrix index
            x=math.cos(angle+pose_theta)*distance
            y=math.sin(angle+pose_theta)*distance

            i = int(self.position_in_map.x + x/self.granularity)
            j = int(self.position_in_map.y + y/self.granularity)

            matrix_index = Cartesian(i, j)

            # get path from robot position to each point on laser
            start = self.position_in_map
            end = matrix_index
            path = Bresenham((start.x, start.y), (end.x, end.y)).path
            # ld.path = path

            # set each point, but the last, on path to free
            for p in path[:-1]:
                l = self.occupancy_map[p[0]][p[1]]
                l.append(self._free)

            # set the position for this laser reading as occupied
            # coloca 1 onde tem uma leitura
            self.map_cell_add_value(i,j,self._occ)


if __name__ == '__main__':
    from listener import Robot

    robot = Robot()
    ogm = OccupancyGridMap()
    robot.start(ogm.update_map)

    # scan = LaserScan()
    # scan.angle_min = -2.35619449615
    # scan.angle_max = 2.35619449615
    # scan.angle_increment = 0.00872664619237
    # scan.ranges = [2.5230000019073486, 2.4820001125335693, 2.4059998989105225, 2.4189999103546143, 2.431999921798706, 2.4179999828338623, 2.4040000438690186, 2.375, 2.365000009536743, 2.325000047683716, 2.302000045776367, 2.311000108718872, 2.2909998893737793, 2.2780001163482666, 2.23799991607666, 2.23799991607666, 2.2290000915527344, 2.2049999237060547, 2.191999912261963, 2.1579999923706055, 2.183000087738037, 2.1459999084472656, 2.128000020980835, 2.122999906539917, 2.0969998836517334, 2.0959999561309814, 2.0759999752044678, 2.0769999027252197, 2.0450000762939453, 2.065999984741211, 2.0490000247955322, 2.0179998874664307, 2.0320000648498535, 2.010999917984009, 2.006999969482422, 2.00600004196167, 1.9709999561309814, 1.9609999656677246, 1.9559999704360962, 1.8990000486373901, 1.847000002861023, 1.4830000400543213, 1.3539999723434448, 1.3450000286102295, 1.3489999771118164, 1.3250000476837158, 1.4149999618530273, 0.9049999713897705, 0.871999979019165, 0.8889999985694885, 0.968999981880188, 1.0290000438690186, 1.1050000190734863, 0.9869999885559082, 1.027999997138977, 0.9819999933242798, 0.8700000047683716, 0.9049999713897705, 0.9340000152587891, 0.9480000138282776, 0.9240000247955322, 0.9139999747276306, 0.8849999904632568, 0.8159999847412109, 0.8109999895095825, 0.8009999990463257, 0.8700000047683716, 0.8370000123977661, 0.8809999823570251, 0.800000011920929, 0.8059999942779541, 0.8059999942779541, 0.8169999718666077, 0.8130000233650208, 0.8059999942779541, 0.8080000281333923, 0.8410000205039978, 0.8510000109672546, 1.159999966621399, 1.2940000295639038, 1.3880000114440918, 1.4320000410079956, 1.4620000123977661, 1.1729999780654907, 1.2649999856948853, 1.3819999694824219, 1.4730000495910645, 1.4800000190734863, 1.5049999952316284, 1.5069999694824219, 1.5190000534057617, 1.524999976158142, 1.5299999713897705, 1.5190000534057617, 1.5130000114440918, 1.5140000581741333, 1.5369999408721924, 1.5169999599456787, 1.5149999856948853, 1.5230000019073486, 1.527999997138977, 1.5390000343322754, 1.5360000133514404, 1.5399999618530273, 1.524999976158142, 1.5369999408721924, 1.531000018119812, 1.5290000438690186, 1.4609999656677246, 1.4049999713897705, 1.5640000104904175, 1.7489999532699585, 1.7569999694824219, 1.7649999856948853, 1.7719999551773071, 1.715000033378601, 1.3389999866485596, 1.2740000486373901, 1.277999997138977, 1.2860000133514404, 1.3179999589920044, 1.4839999675750732, 1.7979999780654907, 1.812000036239624, 1.8049999475479126, 1.8049999475479126, 1.8179999589920044, 1.8289999961853027, 1.8339999914169312, 1.847000002861023, 1.8539999723434448, 1.840000033378601, 1.8480000495910645, 1.8580000400543213, 1.8609999418258667, 1.871999979019165, 1.8960000276565552, 1.9010000228881836, 1.9119999408721924, 1.909999966621399, 1.9229999780654907, 1.9199999570846558, 1.9279999732971191, 1.9359999895095825, 1.9429999589920044, 1.968999981880188, 1.9670000076293945, 1.9809999465942383, 1.9769999980926514, 2.01200008392334, 2.003000020980835, 2.00600004196167, 2.013000011444092, 2.0260000228881836, 2.0350000858306885, 2.061000108718872, 2.078000068664551, 2.1019999980926514, 2.1029999256134033, 2.1080000400543213, 2.1089999675750732, 2.11899995803833, 2.1449999809265137, 2.1559998989105225, 2.1740000247955322, 2.183000087738037, 2.2139999866485596, 2.2139999866485596, 2.2339999675750732, 2.252000093460083, 2.240000009536743, 2.2139999866485596, 2.2639999389648438, 2.311000108718872, 2.3399999141693115, 2.3359999656677246, 2.371999979019165, 2.390000104904175, 2.4240000247955322, 2.424999952316284, 2.438999891281128, 2.453000068664551, 2.496999979019165, 2.5230000019073486, 2.5320000648498535, 2.562999963760376, 2.5999999046325684, 2.1410000324249268, 1.9290000200271606, 1.8899999856948853, 1.8980000019073486, 2.0959999561309814, 2.7269999980926514, 2.756999969482422, 2.7950000762939453, 2.825000047683716, 2.8580000400543213, 2.884999990463257, 2.6730000972747803, 2.312000036239624, 2.312999963760376, 2.3380000591278076, 2.365000009536743, 2.3389999866485596, 2.321000099182129, 2.2799999713897705, 2.247999906539917, 2.4149999618530273, 3.0460000038146973, 3.3570001125335693, 3.4100000858306885, 3.489000082015991, 3.5439999103546143, 3.5810000896453857, 3.6410000324249268, 3.569999933242798, 3.628999948501587, 3.6419999599456787, 3.23799991607666, 2.930999994277954, 2.9140000343322754, 3.26200008392334, 3.5880000591278076, 3.578000068664551, 3.571000099182129, 3.5409998893737793, 3.5250000953674316, 3.5299999713897705, 3.499000072479248, 3.494999885559082, 3.502000093460083, 3.4809999465942383, 3.4649999141693115, 3.4639999866485596, 3.4549999237060547, 3.443000078201294, 3.119999885559082, 2.7980000972747803, 3.0309998989105225, 3.3350000381469727, 2.8610000610351562, 2.5899999141693115, 2.4560000896453857, 2.443000078201294, 2.446000099182129, 2.431999921798706, 2.4189999103546143, 2.4159998893737793, 2.437000036239624, 2.4119999408721924, 2.4030001163482666, 2.3610000610351562, 2.3540000915527344, 2.3940000534057617, 2.4049999713897705, 2.4059998989105225, 2.4130001068115234, 2.4130001068115234, 2.4210000038146973, 2.421999931335449, 2.4260001182556152, 2.4539999961853027, 2.48799991607666, 2.691999912261963, 2.759999990463257, 2.8559999465942383, 3.246000051498413, 3.257999897003174, 3.2639999389648438, 3.257999897003174, 3.2690000534057617, 3.246999979019165, 2.7179999351501465, 2.6740000247955322, 2.696000099182129, 3.0179998874664307, 3.257999897003174, 3.259999990463257, 3.259999990463257, 3.2760000228881836, 3.2690000534057617, 3.2639999389648438, 3.2709999084472656, 3.2730000019073486, 3.2829999923706055, 3.2799999713897705, 3.194000005722046, 2.878000020980835, 2.8540000915527344, 2.8499999046325684, 2.8320000171661377, 2.8289999961853027, 2.861999988555908, 2.869999885559082, 2.871999979019165, 2.875999927520752, 2.874000072479248, 2.878000020980835, 2.9019999504089355, 2.930999994277954, 3.0350000858306885, 3.3410000801086426, 3.385999917984009, 3.3510000705718994, 3.3239998817443848, 3.3919999599456787, 3.3980000019073486, 3.4149999618530273, 3.1519999504089355, 2.8310000896453857, 1.8079999685287476, 1.715000033378601, 1.6790000200271606, 2.010999917984009, 1.8350000381469727, 1.7269999980926514, 1.680999994277954, 1.656000018119812, 1.6080000400543213, 1.590000033378601, 1.5829999446868896, 1.562999963760376, 1.569000005722046, 1.5579999685287476, 1.5479999780654907, 1.5360000133514404, 1.5390000343322754, 1.5420000553131104, 1.5449999570846558, 1.5369999408721924, 1.5369999408721924, 1.531000018119812, 1.5199999809265137, 1.5260000228881836, 1.503999948501587, 1.5219999551773071, 1.527999997138977, 1.5379999876022339, 1.527999997138977, 1.5269999504089355, 1.534000039100647, 1.5460000038146973, 1.5379999876022339, 1.5549999475479126, 1.5640000104904175, 1.5859999656677246, 1.5870000123977661, 1.5709999799728394, 1.0390000343322754, 0.7940000295639038, 0.6940000057220459, 0.6830000281333923, 0.6899999976158142, 0.6790000200271606, 0.6710000038146973, 0.6650000214576721, 0.6620000004768372, 0.6439999938011169, 0.6389999985694885, 0.6430000066757202, 0.6200000047683716, 0.6320000290870667, 0.6370000243186951, 0.621999979019165, 0.6299999952316284, 0.6449999809265137, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6060000061988831, 0.578000009059906, 0.5950000286102295, 0.0, 0.0, 0.6010000109672546, 0.5849999785423279, 0.5789999961853027, 0.597000002861023, 0.5649999976158142, 0.5640000104904175, 0.5550000071525574, 0.5820000171661377, 0.5830000042915344, 0.5559999942779541, 0.5529999732971191, 0.550000011920929, 0.5820000171661377, 0.5699999928474426, 0.5619999766349792, 0.5509999990463257, 0.5809999704360962, 0.5619999766349792, 0.5630000233650208, 0.5789999961853027, 0.5770000219345093, 0.5640000104904175, 0.5839999914169312, 0.5820000171661377, 0.5809999704360962, 0.5910000205039978, 0.5920000076293945, 0.6129999756813049, 0.5979999899864197, 0.6129999756813049, 0.6010000109672546, 0.6079999804496765, 0.5989999771118164, 0.6579999923706055, 1.159000039100647, 1.2660000324249268, 1.3819999694824219, 1.5509999990463257, 1.7419999837875366, 1.8519999980926514, 1.8839999437332153, 1.8880000114440918, 1.8700000047683716, 1.8660000562667847, 1.8660000562667847, 1.8559999465942383, 1.8650000095367432, 1.8550000190734863, 1.8459999561309814, 1.840999960899353, 1.840999960899353, 1.847000002861023, 1.8480000495910645, 1.5809999704360962, 1.3170000314712524, 1.2669999599456787, 1.2660000324249268, 1.2690000534057617, 1.440999984741211, 1.8070000410079956, 1.815999984741211, 1.7680000066757202, 1.7730000019073486, 1.8020000457763672, 1.8170000314712524, 1.8040000200271606, 1.7970000505447388, 1.7960000038146973, 1.7949999570846558, 1.7990000247955322, 1.8040000200271606, 1.8109999895095825, 1.7890000343322754, 1.7610000371932983, 1.371000051498413, 1.340000033378601, 1.409999966621399, 1.2920000553131104, 1.090999960899353, 0.9660000205039978, 0.949999988079071, 0.9419999718666077, 0.9290000200271606, 0.9229999780654907, 0.9200000166893005, 0.9110000133514404, 0.9229999780654907, 0.9079999923706055, 0.9089999794960022, 0.8989999890327454, 0.8980000019073486, 0.8899999856948853, 0.8830000162124634, 0.8889999985694885, 0.9100000262260437, 0.906000018119812, 0.8960000276565552, 0.8999999761581421, 0.8939999938011169, 0.906000018119812, 0.8849999904632568, 0.8999999761581421, 0.8870000243186951, 0.8510000109672546, 0.8659999966621399, 0.8619999885559082, 0.8410000205039978, 0.8489999771118164, 0.8759999871253967, 0.878000020980835, 0.8740000128746033, 0.9070000052452087, 0.9020000100135803, 0.9169999957084656, 0.9039999842643738, 0.9279999732971191, 0.9070000052452087, 0.9200000166893005, 0.9210000038146973, 0.9240000247955322, 0.9290000200271606, 0.9459999799728394, 0.9440000057220459, 0.9440000057220459, 0.9340000152587891, 0.9520000219345093, 0.9549999833106995, 0.9599999785423279, 0.9869999885559082, 1.0290000438690186, 1.2799999713897705, 1.3600000143051147, 1.4170000553131104, 1.4520000219345093, 1.472000002861023, 1.4700000286102295, 1.2879999876022339, 1.3309999704360962, 1.50600004196167, 1.4869999885559082, 1.496999979019165, 1.5140000581741333, 1.503999948501587, 1.5219999551773071, 1.5299999713897705, 1.534000039100647, 1.531000018119812, 1.4199999570846558, 1.3539999723434448, 1.340999960899353, 1.340000033378601, 1.315999984741211, 1.2990000247955322, 1.2979999780654907, 1.2740000486373901, 1.25, 1.2489999532699585, 1.225000023841858, 1.2120000123977661, 1.1950000524520874, 1.1929999589920044, 1.1920000314712524, 1.159000039100647, 1.156000018119812]
    # ogm.update_map((0,0,0), scan)
