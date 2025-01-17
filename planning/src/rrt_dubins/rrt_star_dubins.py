"""
Path planning Sample Code with RRT and Dubins path

author: AtsushiSakai(@Atsushi_twi)

"""

import copy
import math
import os
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
import timeit

#
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../DubinsPath/")
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../RRTStar/")
#
# try:
#     from rrt_star import RRTStar
# except ImportError:
#     raise
from rrt_dubins import dubins_path_planning
from rrt_dubins.rrt_star import RRTStar

show_animation = False


class RRTStarDubins(RRTStar):
    """
    Class for RRT star planning with Dubins path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, glomal_map, rand_area,
                 goal_sample_rate=50,
                 max_iter=150,
                 connect_circle_dist=30.0
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = glomal_map
        self.connect_circle_dist = connect_circle_dist

        self.curvature = 3  # for dubins path
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def planning(self, animation=False, search_until_max_iter=False):
        """
        RRT Star planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            # print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(new_node, self.obstacle_list):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)

            if animation and i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        # print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        # else:
        #     print("Cannot find path")

        return None

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):
        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_length = dubins_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        if len(px) <= 1:  # cannot find a dubins path
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += course_length
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_length = dubins_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        return from_node.cost + course_length

    def get_random_node(self):

        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand),
                            random.uniform(-math.pi, math.pi)
                            )
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.yaw)

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        # print("final")
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path

    def check_collision(self, node, g_map):
        ROBOT_HEIGHT = 0.25
        ROBOT_WIDTH = 0.35

        # Габариты робота в пересчете на ячейки карты
        start_size_x = int(-ROBOT_HEIGHT / (2 * g_map.info.resolution))
        start_size_y = int(-ROBOT_WIDTH / (2 * g_map.info.resolution))
        finish_size_x = -start_size_x
        finish_size_y = -start_size_y

        for i in range(len(node.path_x) - 1):
            x_robot_center = int((node.path_x[i] - g_map.info.origin.position.x) / g_map.info.resolution)
            y_robot_center = int((node.path_y[i] - g_map.info.origin.position.x) / g_map.info.resolution)
            robot_yaw = math.atan2((node.path_y[i + 1] - node.path_y[i]),
                                   (node.path_x[i + 1] - node.path_x[i]))

            sin_yaw = math.sin(robot_yaw)
            cos_yaw = math.cos(robot_yaw)

            if 0 <= x_robot_center + finish_size_x < g_map.info.height and \
                    0 <= y_robot_center + finish_size_y < g_map.info.height:
                for i in range(start_size_x, finish_size_x + 1):
                    for j in range(start_size_y, finish_size_y + 1):

                        x_robot_size = int(x_robot_center + i * cos_yaw + j * sin_yaw)
                        y_robot_size = int(y_robot_center - i * sin_yaw + j * cos_yaw)
                        if int(g_map.data[g_map.info.height * y_robot_size + x_robot_size]) > 75:
                            return False

        return True


def main():
    # print("Start rrt star with dubins planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(0.0)]
    goal = [10.0, 10.0, np.deg2rad(0.0)]

    rrtstar_dubins = RRTStarDubins(start, goal, rand_area=[-2.0, 15.0], global_map=obstacleList)
    path = rrtstar_dubins.planning(animation=show_animation, search_until_max_iter=False)
    if not path:
        return 1
    else:
        if not show_animation:  # pragma: no cover
            rrtstar_dubins.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.001)

            plt.show()
        return 0
    # Draw final path


if __name__ == '__main__':
    number = 10
    result = []
    summa = 0
    for i in range(number):
        starttime = timeit.default_timer()
        k = main()
        result.append(timeit.default_timer() - starttime)
        summa += k
    print('Среднее время', sum(result) / number)
    print('Максимум', max(result))
    print('Минимум', min(result))
    print('Процент ненайденных путей', summa / number * 100)
