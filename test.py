'''
@file: global_planner.py
@breif: global planner application entry
@author: Winter
@update: 2023.3.2
'''
from utils import Grid, Map, SearchFactory
from local_planner import DWA
import numpy as np

if __name__ == '__main__':

    search_factory = SearchFactory()
    # build environment
    start = (2, 2)
    goal = (28, 28)
    env = Grid(51, 31)
    # planner_name = "jps"
    # creat planner
    # planner = DWA(start, goal, env)
    planner = search_factory("jps", start=start, goal=goal, env=env)
    # print(planner.g_planner)
    (cost, path), expand, sum_t = planner.plan()
    points = np.array(path)
    new_array = [list(t) + [0] for t in points]
    for i in range(0,int(len(new_array)),1):
        points1 = new_array[i:i + 2]
        DWAPlan = DWA(points1[0],points1[1],env)
        DWAPlan.run()
    # planner.run()