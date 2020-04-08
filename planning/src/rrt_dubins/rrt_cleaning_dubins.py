"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np
import sys
import timeit
import os
import copy
from rrt_dubins import dubins_path_planning

show_animation = False


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class NodeDubins(Node):
        """
        RRT Node Dubins
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.cost = 0
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5, max_iter=300):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        # expand_dis - максимальная длина линий планирования
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.start_yaw = goal[2]
        self.end_yaw = goal[2]

        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

        self.new_node_list = []

        self.curvature = 1.0  # for dubins path
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)
            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    path = self.generate_final_course(len(self.node_list) - 1)
                    if path is not None:
                        return path

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    # Построение пути
    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def dubins_steer(self, from_node, to_node):

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

    def generate_final_course(self, goal_ind):
        path = [self.end]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(node)

        N = len(path) - 1
        i = 0
        j = N
        new_node_list = []
        path.reverse()
        while i < N:
            if i < j:
                new_node = self.steer(path[i], path[j])

                if self.check_collision(new_node, self.obstacle_list):

                    elem = path[i]
                    next = path[j]
                    if j == N:
                        dubins_node2 = self.NodeDubins(next.x, next.y, self.end_yaw)
                    else:
                        next2 = path[j + 1]
                        dubins_node2 = self.NodeDubins(next.x, next.y,
                                                       math.atan2((next2.y - next.y), (next2.x - next.x)))

                    if i == 0:
                        dubins_node1 = self.NodeDubins(elem.x, elem.y, self.start_yaw)
                    else:
                        dubins_node1 = self.NodeDubins(elem.x, elem.y, math.atan2((next.y - elem.y), (next.x - elem.x)))

                    answer = self.dubins_steer(dubins_node1, dubins_node2)

                    if self.check_collision(answer, self.obstacle_list):
                        new_node_list.append(answer)
                    else:
                        j -= 1
                        continue
                else:
                    j -= 1
                    continue
            # 'это мое нововведение против зацикливания
            if i == j:
                return
            i = j
            j = N

        path = [[self.end.x, self.end.y]]
        for node in new_node_list[::-1]:
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
        path.append([self.start.x, self.start.y])

        new_node_list.insert(0, self.start)
        self.new_node_list = new_node_list
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        # for node in self.node_list:
        #     if node.parent:
        #         plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * size)

        self.plot_start_goal_arrow()
        # plt.plot(self.start.x, self.start.y, "xr")
        # plt.plot(self.end.x, self.end.y, "xr")

        plt.axis("equal")
        plt.axis([-5, 20, -5, 20])
        plt.grid(True)
        plt.pause(0.01)

    def plot_start_goal_arrow(self):  # pragma: no cover
        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start_yaw)
        dubins_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end_yaw)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    # print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]
    # Set Initial parameters
    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(0.0)]
    goal = [10.0, 10.0, np.deg2rad(0.0)]
    rrt = RRT(start,
              goal,
              rand_area=[-2, 15],
              obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    if path is None or len(path) < 2:
        # print("Cannot find path")
        return 1
    else:
        # print(len(path))
        # Draw final path

        if show_animation:
            rrt.draw_graph()

            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.plot([x.x for x in rrt.new_node_list], [y.y for y in rrt.new_node_list], '-v')

            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
        return 0


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
