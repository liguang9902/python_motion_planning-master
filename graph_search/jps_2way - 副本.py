'''
@file: jps.py
@breif: Jump Point Search motion planning
@author: Winter
@update: 2023.1.14
'''
import os, sys
import heapq

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .a_star import AStar
from utils import Env, Node
import time
import math
import numpy as np
from math import factorial
import matplotlib.pyplot as plt


class JPS_2way(AStar):
    '''
    Class for JPS motion planning.

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Env
        environment
    heuristic_type: str
        heuristic function type, default is euclidean

    Examples
    ----------
    >>> from utils import Grid
    >>> from graph_search import JPS
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Grid(51, 31)
    >>> planner = JPS(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
        
    
    def __str__(self) -> str:
        return "Jump Point Search(JPS)"

    def plan(self):
        '''
        JPS motion plan function.
        [1] Online Graph Pruning for Pathfinding On Grid Maps

        Return
        ----------
        path: list
            planning path
        expand: list
            all nodes that planner has searched
        '''
        # OPEN set with priority and CLOSED set
        OPEN_A = []
        OPEN_B = []
        heapq.heappush(OPEN_A, self.start)
        heapq.heappush(OPEN_B, self.goal)
        CLOSED_A = []
        CLOSED_B = []
        global goal_A
        global goal_B
        goal_A = self.goal
        goal_B = self.start
        goal_A.parent = self.goal
        
        goal_B.parent = self.start
        
        sum_t = 0.0
        while OPEN_A and OPEN_B:
            time_start =time.time()
            node_A = heapq.heappop(OPEN_A)
            node_B = heapq.heappop(OPEN_B)

            
            # exists in CLOSED set
            if node_A in CLOSED_A and node_B in CLOSED_B:
                continue
            # if node_B in CLOSED_B:
            #     continue

            # goal found
            if node_A == self.goal :
                CLOSED_A.append(node_A)
                costA, pathA =self.extractPathA(CLOSED_A)
                # pathB.remove(pathB[0])
                # pathA.remove(pathA[0])
                print(pathA)
                return [costA , pathA], CLOSED_A , sum_t
            
            if node_B == self.start :
                CLOSED_B.append(node_B)
                costB, pathB =self.extractPathB(CLOSED_B)
                print(pathB)
                return [ costB, pathB], CLOSED_B, sum_t
            
            ## if ( CLOSED_A and  CLOSED_B):
                
            if node_A in CLOSED_B or node_B in CLOSED_A :#and self.start not in CLOSED_B and self.goal not in CLOSED_A: 
                CLOSED_A.append(node_A)
                # CLOSED_B.append(node_B)

                CLOSED = CLOSED_A + CLOSED_B
                
                # new_CLOSED=list(set(CLOSED))
                # new_CLOSED.sort(key=CLOSED.index)
                
                costA, pathA =self.extractPathA(CLOSED_A)
                costB, pathB =self.extractPathB(CLOSED_B)
                
                
                # set_c = set(pathA) & set(pathB)
                # list_c = list(set_c)
                # if (len(list_c) > 0):
                
                pathA.reverse()
                print(pathA)
                print(pathB)

                path = pathA + pathB
                print(path)
                new_path=list(set(path))
                new_path.sort(key=path.index)
                print(new_path)
                # evaluate_bezier(new_path,50)
                return [ costA + costB, new_path], CLOSED_A + CLOSED_B, sum_t

            jp_list_A = []
            jp_list_B = []
            for motion in self.motions:
                jp_a = self.jump(node_A, motion)
                jp_b = self.jump1(node_B, motion)
                

                # exists and not in CLOSED set
                if jp_a and jp_a not in CLOSED_A:
                    jp_a.parent = node_A.current
                    jp_a.h = self.h(jp_a, goal_A)
                    jp_list_A.append(jp_a)
                    # goal_B = node_A
                    
                if jp_b and jp_b not in CLOSED_B:
                    jp_b.parent = node_B.current
                    jp_b.h = self.hB(jp_b, goal_B)
                    jp_list_B.append(jp_b)
                    # goal_A = node_B
                

            
            for jp_a in jp_list_A:  
                # update OPEN set
                
                heapq.heappush(OPEN_A, jp_a)
                
                # goal found
                if jp_a == goal_A:        
                    break
                # else:
                #     goal_A = node_B
            CLOSED_A.append(node_A)    
            

            for jp_b in jp_list_B :
                # update OPEN set
                
                heapq.heappush(OPEN_B, jp_b)
                
                # goal found
                if jp_b == goal_B:        
                    break
                # else:
                #     goal_B = node_A
                    
            CLOSED_B.append(node_B)
            goal_B = node_A
            goal_A = node_B


            time_end = time.time()
            sum_t = (time_end - time_start)+sum_t
            print('time cost', sum_t, 's')
        return [], []

    def jump(self, node: Node, motion: Node):
        '''
        Jumping search recursively.

        Parameters
        ----------
        node: Node
            current node
        motion: Node
            the motion that current node executes

        Return
        ----------
        jump_point: Node
            jump point or None if searching fails
        '''
        # explore a new node
        new_node = node + motion
        new_node.parent = node.current
        new_node.h = self.h(new_node, goal_A)

        # hit the obstacle
        if new_node.current in self.obstacles:
            return None

        # goal found
        if new_node == goal_A:
            return new_node

        # diagonal
        if motion.current[0] and motion.current[1]:
            # if exists jump point at horizontal or vertical
            x_dir = Node((motion.current[0], 0), None, 1, None)
            y_dir = Node((0, motion.current[1]), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, y_dir):
                return new_node
            
        # if exists forced neighbor
        if self.detectForceNeighbor(new_node, motion):
            return new_node
        else:
            return self.jump(new_node, motion)
        
    def jump1(self, node: Node, motion: Node):
        '''
        Jumping search recursively.

        Parameters
        ----------
        node: Node
            current node
        motion: Node
            the motion that current node executes

        Return
        ----------
        jump_point: Node
            jump point or None if searching fails
        '''
        # explore a new node
        new_node = node + motion
        new_node.parent = node.current
        new_node.h = self.hB(new_node, goal_B)

        # hit the obstacle
        if new_node.current in self.obstacles:
            return None

        # goal found
        if new_node == goal_B:
            return new_node

        # diagonal
        if motion.current[0] and motion.current[1]:
            # if exists jump point at horizontal or vertical
            x_dir = Node((motion.current[0], 0), None, 1, None)
            y_dir = Node((0, motion.current[1]), None, 1, None)
            if self.jump1(new_node, x_dir) or self.jump1(new_node, y_dir):
                return new_node
            
        # if exists forced neighbor
        if self.detectForceNeighbor(new_node, motion):
            return new_node
        else:
            return self.jump1(new_node, motion)
    def detectForceNeighbor(self, node, motion):
        '''
        Detect forced neighbor of node.

        Parameters
        ----------
        node: Node
            current node
        motion: Node
            the motion that current node executes

        Return
        ----------
        flag: bool
            True if current node has forced neighbor else Flase
        '''
        x, y = node.current
        x_dir, y_dir = motion.current

        # horizontal
        if x_dir and not y_dir:
            if (x, y + 1) in self.obstacles and \
                (x + x_dir, y + 1) not in self.obstacles:
                return True
            if (x, y - 1) in self.obstacles and \
                (x + x_dir, y - 1) not in self.obstacles:
                return True
        
        # vertical
        if not x_dir and y_dir:
            if (x + 1, y) in self.obstacles and \
                (x + 1, y + y_dir) not in self.obstacles:
                return True
            if (x - 1, y) in self.obstacles and \
                (x - 1, y + y_dir) not in self.obstacles:
                return True
        
        # diagonal
        if x_dir and y_dir:
            if (x - x_dir, y) in self.obstacles and \
                (x - x_dir, y + y_dir) not in self.obstacles:
                return True
            if (x, y - y_dir) in self.obstacles and \
                (x + x_dir, y - y_dir) not in self.obstacles:
                return True
        
        return False
    def getNeighbor(self, node: Node) -> list:
        '''
        Find neighbors of node.

        Parameters
        ----------
        node: Node
            current node

        Return
        ----------
        neighbors: list
            neighbors of current node
        '''
        return [node + motion for motion in self.motions
                if not self.isCollision(node, node + motion)]

    def extractPathA(self, closed_set):
        '''
        Extract the path based on the CLOSED set.

        Parameters
        ----------
        closed_set: list
            CLOSED set

        Return
        ----------
        cost: float
            the cost of planning path
        path: list
            the planning path
        '''
        cost = 0
        node = closed_set[-1]
        path = [node.current]
        while node != self.start:
            node_parent = closed_set[closed_set.index(Node(node.parent, None, None, None))]
            cost += self.dist(node, node_parent)
            node = node_parent
            path.append(node.current)
        return cost, path

    def extractPathB(self, closed_set):
        '''
        Extract the path based on the CLOSED set.

        Parameters
        ----------
        closed_set: list
            CLOSED set

        Return
        ----------
        cost: float
            the cost of planning path
        path: list
            the planning path
        '''
        cost = 0
        node = closed_set[-1]
        path = [node.current]
        while node != self.goal:
            node_parent = closed_set[closed_set.index(Node(node.parent, None, None, None))]
            cost += self.dist(node, node_parent)
            node = node_parent
            path.append(node.current)
        
        return cost, path
    
    def distanceB(self, node: Node, goal: Node) -> float:
        dx = abs(goal.current[0] - node.current[0])                          #  Diagnol distance 
        dy = abs(goal.current[1] - node.current[1])
        min_xy = min(dx,dy)
        d = dx + dy + (math.sqrt(2) - 2) * min_xy  
        return d

    def hB(self, node: Node, goal: Node) -> float:
            currentDistance = self.distanceB(node, goal)
            startDistance = 1
            if (self.distanceB(self.goal, goal) != 0 ):
                startDistance = self.distanceB(self.goal, goal)

            # 可改进点2：f(n)=g(n)+h(n)*w(n)
            # 动态加权：在放弃搜索最优路径的情况下，使用动态加权来缩短A*搜索的时间
            # 当h较大时，权重系数w也应该较大，此时A*算法会尽快向终点扩展，搜索速度很快但会错过最优路径；
            # 当h较小时，w也应该较小，此时A*算法会倾向于搜索最优路径而减慢搜索速度。
            # if currentDistance > 20:
            #     w = 2.0
            # else:
            #     w = 1.0
            # # w = 1.0    
            w = 1 + currentDistance / startDistance
            return  w * currentDistance
            # return math.hypot(goal.current[0] - node.current[0], goal.current[1] - node.current[1])

    
def comb(n, k):
        return factorial(n) // (factorial(k) * factorial(n-k))

def get_bezier_curve(points):
        n = len(points) - 1
        return lambda t: sum(comb(n, i)*t**i * (1-t)**(n-i)*points[i] for i in range(n+1))

def evaluate_bezier(points, total):
    bezier = get_bezier_curve(points)
    new_points = np.array([bezier(t) for t in np.linspace(0, 1, total)])
    return new_points[:, 0], new_points[:, 1]


