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
    global DWApath
    global DWAcost
    DWAcost = 0 
    DWApath = new_array[0]
    for i in range(0,int(len(new_array)),1):
        points1 = new_array[i:i + 2]
        print(points1[0])
        DWAPlan = DWA(points1[0],points1[1],env)
        history_traj, history_pose = DWAPlan.plan()

        path1 = np.array(history_pose)[:, 0:2]
        cost1 = np.sum(np.sqrt(np.sum(np.diff(path1, axis=0)**2, axis=1, keepdims=True)))
        DWApath += path1
        DWAcost += cost1
    # planner.run()