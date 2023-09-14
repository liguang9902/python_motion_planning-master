'''
@file: graph_search.py
@breif: Base class for planner based on graph searching
@author: Winter
@update: 2023.1.13
'''
import math
import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from utils import Env, Node, Planner

class GraphSearcher(Planner):
    '''
    Base class for planner based on graph searching.

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
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str="euclidean") -> None:
        super().__init__(start, goal, env)
        # heuristic type
        self.heuristic_type = heuristic_type
        # allowed motions
        self.motions = self.env.motions
        # obstacles
        self.obstacles = self.env.obstacles
    
    def distance(self, node: Node, goal: Node) -> float:
        # dx = abs(goal.current[0] - node.current[0])                          #  Diagnol distance 对角线距离
        # dy = abs(goal.current[1] - node.current[1])
        # min_xy = min(dx,dy)
        # d = dx + dy + (math.sqrt(2) - 2) * min_xy  
        # return d
        return math.hypot(goal.current[0] - node.current[0], goal.current[1] - node.current[1])

    def h(self, node: Node, goal: Node) -> float:
        '''
        Calculate heuristic.

        Parameters
        ----------
        node: Node
            current node
        goal: Node
            goal node

        Return
        ----------
        h: float
            heuristic function value of node
        '''
        if self.heuristic_type == "manhattan":
            return abs(goal.current[0] - node.current[0]) + abs(goal.current[1] - node.current[1])
        elif self.heuristic_type == "euclidean":
            return math.hypot(goal.current[0] - node.current[0], goal.current[1] - node.current[1])
        
        elif self.heuristic_type == "test":
            # 可改进点1：启发距离函数也可改进
            # d = math.hypot(goal.current[0] - node.current[0], goal.current[1] - node.current[1])
            # dx = abs(goal.current[0] - node.current[0])                          #  Diagnol distance 
            # dy = abs(goal.current[1] - node.current[1])
            # min_xy = min(dx,dy)
            # currentDistance = dx + dy + (math.sqrt(2) - 2) * min_xy   
            currentDistance = self.distance(node, goal)
            parentD = math.hypot(goal.current[0] - node.parent[0], goal.current[1] - node.parent[1])
            # startDistance = 1
            # if (self.distance(self.start, goal) != 0 ):
            #     startDistance = self.distance(self.start, goal)
            # startDistance = self.distance(self.start, self.goal)

            # 可改进点2：f(n)=g(n)+h(n)*w(n)
            # 动态加权：在放弃搜索最优路径的情况下，使用动态加权来缩短A*搜索的时间
            # 当h较大时，权重系数w也应该较大，此时A*算法会尽快向终点扩展，搜索速度很快但会错过最优路径；
            # 当h较小时，w也应该较小，此时A*算法会倾向于搜索最优路径而减慢搜索速度。
            # if currentDistance > 12:
            #     w = 1.2
            # else:
            #     w = 0.8
            # # w = 1.0    
            # w = 1 + currentDistance / startDistance
            # w = math.exp(max(abs(goal.current[0] - node.current[0]), abs(goal.current[1] - node.current[1])))
            w = math.exp((math.hypot(goal.current[0] - node.current[0], goal.current[1] - node.current[1]) + 
                         max(abs(goal.current[0] - node.current[0]), abs(goal.current[1] - node.current[1])))/ 2)
            return w * currentDistance
        
    
    
    def cost(self, node1: Node, node2: Node) -> float:
        '''
        Calculate cost for this motion.
        '''
        if self.isCollision(node1, node2):
            return float("inf")
        return self.dist(node1, node2) 
        # return self.distance(node1, node2)修改点
    

    def isCollision(self, node1: Node, node2: Node) -> bool:
        '''
        Judge collision when moving from node1 to node2.

        Parameters
        ----------
        node1, node2: Node

        Return
        ----------
        collision: bool
            True if collision exists else False
        '''
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return True

        x1, y1 = node1.current
        x2, y2 = node2.current

        if x1 != x2 and y1 != y2:
            if x2 - x1 == y1 - y2:
                s1 = (min(x1, x2), min(y1, y2))
                s2 = (max(x1, x2), max(y1, y2))
            else:
                s1 = (min(x1, x2), max(y1, y2))
                s2 = (max(x1, x2), min(y1, y2))
            if s1 in self.obstacles or s2 in self.obstacles:
                return True
        return False
