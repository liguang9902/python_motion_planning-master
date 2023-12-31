'''
@file: global_planner.py
@breif: global planner application entry
@author: Winter
@update: 2023.3.2
'''
import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))
from utils import Grid, Map, SearchFactory, Plot


if __name__ == '__main__':
    '''
    path searcher constructor
    '''
    search_factory = SearchFactory()
    
    '''
    graph search
    '''
    # build environment0
    start = (2, 2)
    goal = (28, 28)
    env = Grid(31, 31)

    # Plot(start, goal, env).plotEnv("test map")
    # creat planner
    # planner = search_factory("a_star", start=start, goal=goal, env=env)
    # planner = search_factory("dijkstra", start=start, goal=goal, env=env)
    # planner = search_factory("gbfs", start=start, goal=goal, env=env)
    # planner = search_factory("jps", start=start, goal=goal, env=env)
    # planner = search_factory("d_star", start=start, goal=goal, env=env)
    # planner = search_factory("lpa_star", start=start, goal=goal, env=env).
    # planner = search_factory("d_star_lite", start=start, goal=goal, env=env)
    planner = search_factory("JPS_2way", start=start, goal=goal, env=env)

    # animation
    planner.run()

    # ========================================================

    '''
    sample search
    '''
    # # build environment
    # start = (18, 8)
    # goal = (37, 18)
    # env = Map(51, 31)

    # # creat planner
    # planner = search_factory("rrt", start=start, goal=goal, env=env, max_dist=0.5, sample_num=10000)
    # # planner = search_factory("rrt_connect", start=start, goal=goal, env=env, max_dist=0.5, sample_num=10000)
    # # planner = search_factory("rrt_star", start=start, goal=goal, env=env, max_dist=0.5, r=10, sample_num=10000)
    # # planner = search_factory("informed_rrt", start=start, goal=goal, env=env, max_dist=0.5, r=12, sample_num=1500)

    # # animation
    # planner.run()