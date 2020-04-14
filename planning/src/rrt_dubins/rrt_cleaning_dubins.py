"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random
from itertools import tee

import matplotlib.pyplot as plt
import numpy as np
import timeit
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

    def __init__(self, start, goal, glomal_map, rand_area, obstacles,
                 expand_dis=1, path_resolution=0.5, goal_sample_rate=5, max_iter=100):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]гр. 635 сообщение получила

        """
        # expand_dis - максимальная длина линий планирования

        self.obstacles = obstacles

        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.start_yaw = start[2]
        self.end_yaw = goal[2]

        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.global_map = glomal_map
        self.node_list = []

        self.new_node_list = []

        self.curvature = 3.0  # for dubins path
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def planning(self, animation=False):
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

            # starttime = timeit.default_timer()
            if self.check_collision(new_node, self.global_map):
                self.node_list.append(new_node)
            # print(timeit.default_timer() - starttime)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.global_map):
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

                if self.check_collision(new_node, self.global_map):

                    current_elem = path[i]
                    next_elem = path[j]

                    if i == 0:
                        start_angle = self.start_yaw
                    else:
                        start_angle = new_node_list[-1].path_yaw[-1]

                    if j == N:
                        end_angle = self.end_yaw
                    else:
                        next2_elem = path[j + 1]
                        end_angle = math.atan2((next2_elem.y - next_elem.y), (next2_elem.x - next_elem.x))

                    dubins_node1 = self.NodeDubins(current_elem.x, current_elem.y, start_angle)
                    dubins_node2 = self.NodeDubins(next_elem.x, next_elem.y, end_angle)

                    result_path = self.dubins_steer(dubins_node1, dubins_node2)
                    if self.check_collision(result_path, self.global_map):
                        new_node_list.append(result_path)
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

        path = [[self.end.x, self.end.y, self.end_yaw]]
        for node in new_node_list[::-1]:
            for (ix, iy, iyaw) in zip(reversed(node.path_x),
                                      reversed(node.path_y),
                                      reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
        path.append([self.start.x, self.start.y, self.start_yaw])
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            nearest_ind = self.get_nearest_node_index(self.node_list, self.end)
            nearest_node = self.node_list[nearest_ind]
            radius = math.hypot(abs(nearest_node.y - self.end.y), abs(nearest_node.y - self.end.x))

            # rnd = self.Node(random.uniform(nearest_node.x - self.end.x/2, self.end.x + self.end.x/2),
            #                 random.uniform(nearest_node.y - self.end.x/2,self.end.y + self.end.x/2))

            rnd = self.Node(random.uniform(self.end.x - 1.5*radius, self.end.x + 1.5*radius),
                            random.uniform(self.end.y - 1.5*radius, self.end.y + 1.5*radius))

            # rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
            #                 random.uniform(self.min_rand, self.max_rand))
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
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        # copy_obstacleList, _ = tee(self.obstacle_list, 2)
        for p in self.obstacles:
            ox = p.x
            oy = p.y
            size = 0.35/2
            plt.plot(ox, oy, "ok", ms=30 * size)

        self.plot_start_goal_arrow()
        # plt.plot(self.start.x, self.start.y, "xr")
        # plt.plot(self.end.x, self.end.y, "xr")

        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
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
    def check_collision00(node, obstacleList):

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
    def check_collision11(node, obstacleList):

        # self.draw_graph()

        if node is None:
            return False

        ROBOT_HEIGHT = 0.25
        ROBOT_WIDTH = 0.35

        for (x, y) in zip(node.path_x, node.path_y):
            nearest_obstacles = filter(lambda ob: x - ROBOT_HEIGHT <= ob.x <= x + ROBOT_HEIGHT and
                                                  y - ROBOT_WIDTH <= ob.y <= y + ROBOT_WIDTH, obstacleList)
            for p in nearest_obstacles:
                ox = p.x
                oy = p.y
                dx_list = ox - x
                dy_list = oy - y
                d_list = dx_list * dx_list + dy_list * dy_list

                if d_list <= ROBOT_WIDTH ** 2:
                    return False  # collision

        return True  # safe

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

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
