"""
Plot tools 2D
@author: huiming zhou
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import factorial
from decimal import *

from .env import Env, Grid, Map, Node


class Plot:
    def __init__(self, start, goal, env: Env):
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        self.env = env
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()

    def animation(self, path, name,cost=None, expand=None, history_pose=None):
        name = name + "\ncost: " + str(cost) if cost else name
        self.plotEnv(name)
        if expand:
            self.plotExpand(expand)
        if history_pose:
            self.plotHistoryPose(history_pose)
        self.plotPath(path)
        plt.show()

    def animation1(self, path, name,cost=None, expand=None, history_pose=None,time_S=None):
        name = name + "\ncost: " + str(cost) if cost else name
        name = name + "\ntime: " + str(time_S) + "s"
        self.plotEnv(name)
        if expand:
            self.plotExpand(expand)
        if history_pose:
            self.plotHistoryPose(history_pose)
        self.plotPathBez(path)
        plt.show()

    def plotEnv(self, name):
        plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")

        if isinstance(self.env, Grid):
            obs_x = [x[0] for x in self.env.obstacles]
            obs_y = [x[1] for x in self.env.obstacles]
            plt.plot(obs_x, obs_y, "sk")

        if isinstance(self.env, Map):
            ax = self.fig.add_subplot()
            # boundary
            for (ox, oy, w, h) in self.env.boundary:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )
            # rectangle obstacles
            for (ox, oy, w, h) in self.env.obs_rect:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )
            # circle obstacles
            for (ox, oy, r) in self.env.obs_circ:
                ax.add_patch(patches.Circle(
                        (ox, oy), r,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )

        plt.title(name)
        plt.axis("equal")

    def plotExpand(self, expand):
        if self.start in expand:
            expand.remove(self.start)
        if self.goal in expand:
            expand.remove(self.goal)

        count = 0
        if isinstance(self.env, Grid):
            for x in expand:
                count += 1
                plt.plot(x.current[0], x.current[1], color="#dddddd", marker='s')
                plt.gcf().canvas.mpl_connect('key_release_event',
                                            lambda event: [exit(0) if event.key == 'escape' else None])
                if count < len(expand) / 3:         length = 20
                elif count < len(expand) * 2 / 3:   length = 30
                else:                               length = 40
                if count % length == 0:             plt.pause(0.001)
        
        if isinstance(self.env, Map):
            for x in expand:
                count += 1
                if x.parent:
                    plt.plot([x.parent[0], x.current[0]], [x.parent[1], x.current[1]], 
                        color="#dddddd", linestyle="-")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)

        plt.pause(0.01)

    def plotPath(self, path) -> None:
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        plt.plot(path_x, path_y, linewidth='2', color='#13ae00')
        plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")

    def plotPathBez(self, path) -> None:
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        points = np.array(path)
        n = len(points) 
        print(n)
        # points1 = points[0:int(n/4) ]
        # points2 = points[int(n/4) - 1: int(n/2) ]
        # points3 = points[int(n/2) - 1: int(n/4 * 3) ]
        # points4 = points[int(n/4 * 3) - 1: int(n) ]
        # points1 = np.array([path[0],path[2],path[3],path[6],path[7],path[8],path[10],path[12]])
        # b_x, b_y = evaluate_bezier(points,100)
        # b_x1, b_y1 = evaluate_bezier(points1,100)
        # b_x2, b_y2 = evaluate_bezier(points2,100)
        # b_x3, b_y3 = evaluate_bezier(points3,100)
        # b_x4, b_y4 = evaluate_bezier(points4,100)
        plt.plot(path_x, path_y, linewidth='2', color='#13ae00')
        # plt.plot(b_x, b_y, linewidth='2', color='#ffff00')
        # plt.plot(b_x1, b_y1, linewidth='2', color='#ffff00')
        # plt.plot(b_x2, b_y2, linewidth='2', color='#ffff00')
        # plt.plot(b_x3, b_y3, linewidth='2', color='#ffff00')
        # plt.plot(b_x4, b_y4, linewidth='2', color='#ffff00')
        
        newpoint = np.array([path[0]])
        for i in range(0,int(len(points)),2):
        # while  i in int(len(points))+1:   
            points1 = points[i:i + 3]
           
            b_x1, b_y1 = evaluate_bezier(points1,100)      
            plt.plot(b_x1, b_y1, linewidth='2', color='#ffff00')
        plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")

    def plotAgent(self, pose: tuple, radius: float=1) -> None:
        x, y, theta = pose
        ref_vec = np.array([[radius / 2], [0]])
        rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta),  np.cos(theta)]])
        end_pt = rot_mat @ ref_vec + np.array([[x], [y]])

        try:
            self.ax.artists.pop()
            for art in self.ax.get_children():
                if isinstance(art, matplotlib.patches.FancyArrow):
                    art.remove()
        except:
            pass

        self.ax.arrow(x, y, float(end_pt[0]) - x, float(end_pt[1]) - y,
                width=0.1, head_width=0.40, color="r")
        circle = plt.Circle((x, y), radius, color="r", fill=False)
        self.ax.add_artist(circle)

    def plotHistoryPose(self, history_pose):
        count = 0
        for pose in history_pose:
            if count < len(history_pose) - 1:
                plt.plot([history_pose[count][0], history_pose[count + 1][0]],
                    [history_pose[count][1], history_pose[count + 1][1]], c="r")
            count += 1
            self.plotAgent(pose)
            plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event: [exit(0) if event.key == 'escape' else None])
            if count < len(history_pose) / 3:         length = 5
            elif count < len(history_pose) * 2 / 3:   length = 10
            else:                                     length = 20
            if count % length == 0:             plt.pause(0.01)

    def connect(self, name: str, func) -> None:
        self.fig.canvas.mpl_connect(name, func)

    def clean(self):
        plt.cla()

    def update(self):
        self.fig.canvas.draw_idle()

    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl
    
def comb(n, k):
        return factorial(n) // (factorial(k) * factorial(n-k))

def get_bezier_curve(points):
        n = len(points) - 1
        return lambda t: sum(comb(n, i)*t**i * (1-t)**(n-i)*points[i] for i in range(n+1))

def evaluate_bezier(points, total):
    bezier = get_bezier_curve(points)
    new_points = np.array([bezier(t) for t in np.linspace(0, 1, total)])
    return new_points[:, 0], new_points[:, 1]