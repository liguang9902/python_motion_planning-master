'''
@file: env.py
@breif: 2-dimension environment and graph node
@author: Winter
@update: 2023.1.13
'''
from math import sqrt
from abc import ABC, abstractmethod
import numpy as np
import random

class Node(object):
    '''
    Class for searching nodes.

    Parameters
    ----------
    current: tuple
        current coordinate
    parent: tuple
        coordinate of parent node
    g: float
        path cost
    h: float
        heuristic cost

    Examples
    ----------
    >>> from env import Node
    >>> node1 = Node((1, 0), (2, 3), 1, 2)
    >>> node2 = Node((1, 0), (2, 5), 2, 8)
    >>> node3 = Node((2, 0), (1, 6), 3, 1)
    ...
    >>> node1 + node2
    >>> Node((2, 0), (2, 3), 3, 2)
    ...
    >>> node1 == node2
    >>> True
    ...
    >>> node1 != node3
    >>> True
    '''
    def __init__(self, current: tuple, parent: tuple, g: float, h: float) -> None:
        self.current = current
        self.parent = parent
        self.g = g
        self.h = h
    
    def __add__(self, node):
        return Node((self.current[0] + node.current[0], self.current[1] + node.current[1]), 
                     self.parent, self.g + node.g, self.h)

    def __eq__(self, node) -> bool:
        return self.current == node.current
    
    def __ne__(self, node) -> bool:
        return not self.__eq__(node)

    def __lt__(self, node) -> bool:
        return self.g + self.h < node.g + node.h or \
                (self.g + self.h == node.g + node.h and self.h < node.h)

    def __str__(self) -> str:
        return "----------\ncurrent:{}\nparent:{}\ng:{}\nh:{}\n----------" \
            .format(self.current, self.parent, self.g, self.h)

class Env(ABC):
    '''
    Class for building 2-d workspace of robots.

    Parameters
    ----------
    x_range: int
        x-axis range of enviroment
    y_range: int
        y-axis range of environmet

    Examples
    ----------
    >>> from utils import Env
    >>> env = Env(30, 40)
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range

    @property
    def grid_map(self) -> set:
        return {(i, j) for i in range(self.x_range) for j in range(self.y_range)}

    @abstractmethod
    def init(self) -> None:
        pass

class Grid(Env):
    '''
    Class for discrete 2-d grid map.
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
        # allowed motions
        # 可改进点3：增加算法搜索的临域：
        self.motions = [Node((-1, 0), None, 1, None), Node((-1, 1),  None, sqrt(2), None),
                        Node((0, 1),  None, 1, None), Node((1, 1),   None, sqrt(2), None),
                        Node((1, 0),  None, 1, None), Node((1, -1),  None, sqrt(2), None),
                        Node((0, -1), None, 1, None), Node((-1, -1), None, sqrt(2), None)]
        # obstacles
        self.obstacles = None
        self.init()
    
    def init(self) -> None:
        '''
        Initialize grid map.
        '''
        x, y = self.x_range, self.y_range
        obstacles = set()

        # boundary of environment
        for i in range(x):
            obstacles.add((i, 0))
            obstacles.add((i, y - 1))
        for i in range(y):
            obstacles.add((0, i))
            obstacles.add((x - 1, i))

        # user-defined obstacles
        # for i in range(3, 15):
        #     obstacles.add((i, 10))
        # for i in range(7, 11):
        #     obstacles.add((3, i))
        # for i in range(4, 16):
        #     obstacles.add((i, 2))
        # for i in range(3, 7):
        #     obstacles.add((7, i))
        # for i in range(19, 27):
        #     obstacles.add((i, 12))
        # for i in range(2, 7):
        #     obstacles.add((20, i))
        # obstacles.add((21, 2))
        # obstacles.add((22, 2))
        # for i in range(14, 18):
        #     obstacles.add((5, i))
        #     obstacles.add((13, i))
        # for i in range(5, 9):
        #     obstacles.add((25, i))
        #     obstacles.add((26, i))
        # for i in range(15, 18):
        #     obstacles.add((23, i))
        #     obstacles.add((24, i))
        # for i in range(10, 17):
        #     obstacles.add((25, i))
        # for i in range(42, 50):
        #     obstacles.add((i, 10))
        # for i in range(32, 45):
        #     obstacles.add((i, 20))  
        #常用障碍  
        for i in range(10, 21):
            obstacles.add((i, 15))
        for i in range(15):
            obstacles.add((20, i))
        for i in range(15, 30):
            obstacles.add((30, i))
        for i in range(16):
            obstacles.add((40, i))

        # user-defined obstacles 设定用30*30
        # for i in range(2,4):
        #     obstacles.add((i,5))
        #     obstacles.add((i,6))
        # for i in range(3,6):
        #     obstacles.add((6,i))
        #     obstacles.add((7,i))
        # for i in range(7,9):
        #     obstacles.add((7,i))
        #     obstacles.add((8,i))
        # for i in range(8,14):
        #     obstacles.add((i,12))
        #     obstacles.add((i,13))
        #     obstacles.add((i,14))
        # for i in range(9,11):
        #     obstacles.add((10,i))
        #     obstacles.add((11,i))
        # for i in range(16,20):
        #     obstacles.add((16,i))
        #     obstacles.add((17,i))    
        #     obstacles.add((18,i))    
        #     obstacles.add((19,i)) 
        # for i in range(22,28):
        #     obstacles.add((i,23))
        # for i in range(12,21):
        #     obstacles.add((24,i))
        #     obstacles.add((25,i)) 
        # for i in range(1,8):
        #     obstacles.add((14,i))
        #     obstacles.add((15,i)) 
        # for i in range(18,21):
        #     obstacles.add((i,11))
        #     obstacles.add((i,12))
        # for i in range(23,25):
        #     obstacles.add((i,26))
        #     obstacles.add((i,27))
        # for i in range(20,27):
        #     obstacles.add((i,7))
        #     obstacles.add((i,8))
        # for i in range(19,22):
        #     obstacles.add((i,2))
        #     obstacles.add((i,3))
        #     obstacles.add((i,4))
        # for i in range(1,4):
        #     obstacles.add((27,i))
        # for i in range(27,30):
        #     obstacles.add((i,10))
        # for i in range(12,18):
        #     obstacles.add((i,25))
        #     obstacles.add((i,24))
        #     obstacles.add((i,23))
        # for i in range(20,30):
        #     obstacles.add((4,i))
        #     obstacles.add((5,i))
        # for i in range(1,6):
        #     obstacles.add((i,10))
        # for i in range(3,8):
        #     obstacles.add((i,17))
        #     obstacles.add((i,18))
        # for i in range(16,22):
        #     obstacles.add((10,i))
        #     obstacles.add((11,i))
        # for i in range(2,4):
        #     obstacles.add((i,14))
        #     obstacles.add((i,13))
        # for i in range(8,14):
        #     obstacles.add((i,29))
        #     obstacles.add((i,28))
        # for i in range(24,30):
        #     obstacles.add((19,i))
        #     obstacles.add((20,i))

        #长条形
        # for i in range(1,17):
        #     obstacles.add((7,i))
        # for i in range(14,30):
        #     obstacles.add((19,i))  

        # for i in range(1,15):
        #     obstacles.add((i,24)) 
        # for i in range(19,23):
        #     obstacles.add((i,20)) 
        # for i in range(11,30):
        #     obstacles.add((i,7))


        # for i in range(int((x-1)*(y-1)/10)):
        #     j = random.randint(1,x-1)
        #     k = random.randint(1,y-1)
        #     obstacles.add((j,k))
        self.obstacles = obstacles

    def update(self, obstacles):
        self.obstacles = obstacles 


class Map(Env):
    '''
    Class for continuous 2-d map.
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
        self.boundary = None
        self.obs_circ = None
        self.obs_rect = None
        self.init()

    def init(self):
        '''
        Initialize map.
        '''
        x, y = self.x_range, self.y_range

        # boundary of environment
        self.boundary = [
            [0, 0, 1, y],
            [0, y, x, 1],
            [1, 0, x, 1],
            [x, 1, 1, y]
        ]

        # user-defined obstacles
        self.obs_rect = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]

        self.obs_circ = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

    def update(self, boundary, obs_circ, obs_rect):
        self.boundary = boundary if boundary else self.boundary
        self.obs_circ = obs_circ if obs_circ else self.obs_circ
        self.obs_rect = obs_rect if obs_rect else self.obs_rect
