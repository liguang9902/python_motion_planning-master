'''
@file: local_planner.py
@breif: Base class for local planner.
@author: Winter
@update: 2023.3.2
'''
import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from graph_search import *
from sample_search import *


class SearchFactory(object):
    def __init__(self) -> None:
        pass

    def __call__(self, planner_name, **config):
        if planner_name == "a_star":
            return AStar(**config)
        elif planner_name == "dijkstra":
            return Dijkstra(**config)
        elif planner_name == "gbfs":
            return GBFS(**config)
        elif planner_name == "jps":
            return JPS(**config)
        elif planner_name == "d_star":
            return DStar(**config)
        elif planner_name == "lpa_star":
            return LPAStar(**config)
        elif planner_name == "d_star_lite":
            return DStarLite(**config)
        elif planner_name == "rrt":
            return RRT(**config)
        elif planner_name == "rrt_connect":
            return RRTConnect(**config)
        elif planner_name == "rrt_star":
            return RRTStar(**config)
        elif planner_name == "informed_rrt":
            return InformedRRT(**config)  
        elif planner_name == "JPS_2way":
            return JPS_2way(**config)
        else:
            raise ValueError("The `planner_name` must be set correctly.")