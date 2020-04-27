import copy
from math import atan2, cos, degrees, inf, pi, radians, sin, sqrt

import matplotlib
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import traci
from matplotlib import patches
from matplotlib.path import Path
from matplotlib.pyplot import Axes
from shapely.geometry import LineString, MultiPoint, Point, Polygon
from shapely.ops import nearest_points

import HelperFcts as hf
import Paths as params
import Sumo.Plotting as splt
from HelperFcts import Colours
from PathPlanningTools import DubinsCurves
from Sumo import SumoHelperFcts as shf
from Sumo import SumoParameters as sp

# ------------------------ DEFINES ----------------------
matplotlib.use("TkAgg")
color = hf.Colours()
# -------------------------------------------------------


def calc_node_dist(node1, node2):
    """
    Calculates the distance between two Nodes.

    :param node1: Starting Node
    :type node1: Node
    :param node2: Ending Node
    :type node2: Node
    :return: float
    """
    return sqrt((node2.y - node1.y) ** 2 + (node2.x - node1.x) ** 2)


def node_in_circle(centre, rad, node):
    """
    Checks whether a node is within a circle

    :param centre: centre of the circle
    :type centre: Node
    :param rad: radius of the circle
    :type rad: float
    :param node: node to be checked
    :type node: Node
    :return: bool
    """
    if calc_node_dist(centre, node) < rad:
        return True
    else:
        return False


class Node():
    """
    Base Node Class for path finding algorithms
    """
    def __init__(self, x=0, y=0, direction=0.0, ttype=None, z=0):
        """
        Node Constructor

        :param z: z position
        :type z: float
        :param x: x position
        :type x: float
        :param y: y position
        :type y: float
        :param direction: direction of the node, starting from the positive x-axis in a CCW direction
                          given in radians
        :type direction: float
        :param ttype: type of the node (start, nearest, new, fixed) giving the plot style
        :type ttype: str
        """
        self.nbr = 0
        self.lane_id = None
        self.x = x
        self.y = y
        self.cost = 0
        self.direction = direction
        self.parent = None
        self.child = None
        self.path = None
        self.label = None
        self.f = 0
        self.g = 0
        self.length = 0
        if not ttype:
            self.type = 'fixed'
        else:
            self.type = ttype

    # def __ne__(self, other):
    #     if self.x == other.x and self.y == other.y:
    #         return True
    #     else:
    #         return False

    def set_parent(self, parent, edge=None):
        """
        Assigns the parent to a given node. In case of Dubins Trees, the connecting path is also added to the node.

        :param parent: parent node
        :type parent: Node
        :param edge: the connecting dubins path from parent to self
        :type edge: DubinsEdge
        """
        self.parent = parent
        if edge:
            self.path = edge.segment
            self.x = edge.segment[0][-1]
            self.y = edge.segment[1][-1]
            self.direction = edge.segment[2][-1]
            self.calc_cost(edge.length)
        else:
            self.calc_cost()

    def set_child(self, kid):
        print("In Function!")
        self.child = kid

    def set_kiddo(self, kid):
        self.child = kid

    def calc_cost(self, segment_length=None):
        """
        Calculates the cost of a node, given as the distance from parent to self

        :param segment_length: length of the dubins path
        :type segment_length: float
        """
        if self.parent:
            if segment_length:
                self.cost = self.parent.cost + segment_length
            else:
                self.cost = self.parent.cost + calc_node_dist(self, self.parent)
        else:
            self.cost = 0

    # def draw(self, surf, radius=10, clr=color.light_blue):
    #     """
    #     Draws the Node onto a given pygame surface (deprecated)
    #
    #     :param surf: pygame surface to be drawn on
    #     :type surf: Surface
    #     :param radius: the radius of the drawn circle
    #     :type radius: float
    #     :param clr: color of the drawn circle
    #     :type clr: Colours
    #     """
    #     pygame.draw.circle(surf, clr, (int(self.x), int(self.y)), radius)
    #     pygame.draw.circle(surf, color.black, (int(self.x), int(self.y)), radius, 1)
    #     self.label = pygame.font.SysFont("monospace", 15).render(str(self.nbr), 1, color.black)

    def plot(self, clr, ax, r=.3, fill=True, annotate=False):
        """
        Plots the node into a given plot data series.

        :param annotate: adds an annotation with the point coordinates to the plot
        :type annotate: bool
        :param clr: color of the node
        :type clr: Colours
        :param ax: given plot data series
        :type ax: Axes
        :param r: radius of the plotted circle
        :type r: float
        :param fill: makes the circle filled (True) or just draws the outline (False)
        :type fill: bool
        :return: returns the matplot patch
        :rtype: patches.Circle
        """
        if not fill:
            node = matplotlib.patches.Circle((self.x, self.y), r, fill=False, edgecolor=clr)
        else:
            node = matplotlib.patches.Circle((self.x, self.y), r, facecolor=clr, zorder=8)
        ax.add_patch(node)

        if annotate:
            x_coord = self.x + r + 1
            y_coord = self.y + r + 1
            ax.annotate(self.nbr, (x_coord, y_coord), size=7)
        return node

    def redirect(self, target):
        """
        Changes the direction of a node to a target

        :param target: target node fo direction
        :type target: Node
        """
        self.direction = atan2((target.y - self.y), (target.x - self.x))

    def set_length_and_lane_id(self, length, lane_id):
        """
        Sets length and lane_id values for the node. These values are used within the A* navigation.

        :param lane_id: The ID of the assigned lane
        :type lane_id: str
        :param length: length of the assigned lane
        :type length: float
        """
        self.lane_id = lane_id
        self.length = length


class Edge:
    """
    Edge Class for straight edges to be used in path finding algorithms
    """
    def __init__(self, start=None, end=None):
        """
        Edge Constructor

        :param start: start node of the edge
        :type start: Node
        :param end: end node of the edge
        :type end: Node
        """
        self.length = 0
        self.start_node = start
        self.end_node = end
        self.evaluation_pts = []
        self.calc_length()

    def get_edge_as_list(self):
        """
        Returns the edge as list, to simplify plotting

        :return: edge as list
        :rtype: list[float]
        """
        return [self.start_node.x, self.start_node.y, self.end_node.x, self.end_node.y]

    def get_edge_as_linestring(self):
        return LineString([[self.start_node.x, self.start_node.y], [self.end_node.x, self.end_node.y]])

    def calc_length(self):
        """
        Calculates the length of the edge
        """
        self.length = calc_node_dist(self.start_node, self.end_node)

    # def draw(self, surf):
    #     """
    #     Draws the edge onto a given pygame surface
    #
    #     :param surf: The pygame surface to be plotted on
    #     :type surf: Surface
    #     """
    #     pygame.draw.line(surf, color.black, (int(self.start_node.x), int(self.start_node.y)),
    #                      (int(self.end_node.x), int(self.end_node.y)))

    def plot(self, clr, ax):
        """
        Plots the edge

        :param clr: Color of the edge
        :type clr: hf.Colours
        :param ax: plot data series
        :type ax: plt.Axes
        """
        x = [self.start_node.x , self.end_node.x]
        y = [self.start_node.y, self.end_node.y]
        ax.plot(x, y, color=clr)

    def get_edge_as_linestring(self):
        return LineString([[self.start_node.x, self.start_node.y], [self.end_node.x, self.end_node.y]])


class Tree:
    """
    Tree class to be used in path finding algorithms. Used for trees with straight edges
    """
    def __init__(self, start, goal, rad, ax):
        """
        Tree Constructor

        :param start: start node of the tree
        :type start: Node
        :param goal: goal node of the tree, which will be tried to reach in a path planning problem
        :type goal: Node
        :param rad: radius around the goal node, if a node reaches the resulting circle, the goal counts as reached
        :type rad: float
        :param ax: plot data series in case of plotting
        :type ax: Axes
        """
        self.goal_node = goal
        self.start_node = start
        self.nodes = []
        self.beacons = []
        if start:
            self.insert_node(start, ax)
        if goal and ax:
            self.goal_node.plot(color.tree_start_nd, ax, r=rad)
        self.rewired = False
        self.target_nodes = []

    def get_node_by_nbr(self, nbr):
        """
        Gets a not with the given number in the tree, if exist

        :param nbr: number of the node to be found
        :type nbr: int
        :return: return the node wit the given number
        :rtype: Node
        """
        for node in self.nodes:
            if node.nbr == nbr:
                return node
        return None

    def get_closest_in_target_path(self, new_node):
        dist = inf
        closest = None
        for node in self.target_nodes:
            new_dist = calc_node_dist(node, new_node)
            if new_dist < dist:
                dist = new_dist
                closest = node
        return closest

    def plot(self, ax, new_node=None, remove_existing=False, style='-'):
        """
        Draws the tree to a given plot data series

        :param ax: the plot data series
        :type ax: Axes
        :param new_node: If a new node is given, only the new edge is plotted
        :type new_node: Node
        """
        if self.rewired or not new_node:
            if remove_existing:
                for i in range(0, len(ax.lines)):
                    ax.lines.pop(-1)
            for node in self.nodes:
                if node.parent:
                    ax.plot([node.parent.x, node.x], [node.parent.y, node.y], style, linewidth='.7')
        else:
            ax.plot([new_node.parent.x, new_node.x], [new_node.parent.y, new_node.y], style, linewidth='.7')

    def insert_node(self, node, ax):
        """
        Adds a new node to the tree

        :param node: the new node to be added
        :type node: Node
        :param ax: the plot data series in case of plotting
        :type ax: Axes
        :return: if plotted the new node plot patch is returned
        :rtype: patches.Circle
        """
        self.nodes.append(node)
        if node.type == 'start':
            clr = color.tree_start_nd
        elif node.type == 'nearest':
            clr = color.tree_nearest_nd
        elif node.type == 'new':
            clr = color.tree_new_nd
        else:
            clr = color.tree_nd

        if ax:
            new_node = node.plot(clr, ax)
            return new_node

    def plot_target_path(self, successful_node, ax):
        """
        Plots the target path into a given plot data series

        :param successful_node: node that reached the goal
        :type successful_node: Node
        :param ax: plot data series
        :type ax: Axes
        """
        print("Target found!")
        print("Total cost: ", successful_node.cost)
        while successful_node.parent:
            successful_node.plot(color.tree_nearest_nd, ax)
            ax.plot([successful_node.parent.x, successful_node.x], [successful_node.parent.y, successful_node.y],
                    color=color.tree_nearest_nd, linewidth='1', zorder=7)
            successful_node = successful_node.parent

    def renumber_target_path(self, successful_node):
        """
        Renumbers the nodes along the target path, starting with 0 being the number of the start node and
        number_of_nodes being the number of the node within the target radius.

        :param successful_node: node within the target radius
        :type successful_node: Node
        """
        for node in self.nodes:
            node.nbr = inf

        nbr_of_nodes = 0
        end_node = successful_node
        while end_node.parent:
            nbr_of_nodes += 1
            end_node = end_node.parent

        i = 1
        while successful_node.parent:
            self.target_nodes.append(successful_node)
            successful_node.nbr = nbr_of_nodes + 1 - i
            successful_node = successful_node.parent
            i += 1




    def find_node_closest_to_goal(self):
        """
        Finds the node in the tree, that is closest to the goal node. The direct distance is used and not the length
        of the dubins edge.

        :return: closest tree node to goal
        :rtype: Node
        """
        current_dist = inf
        nearest = Node()
        for node in self.nodes:
            new_dist = calc_node_dist(node, self.goal_node)
            if new_dist < current_dist:
                current_dist = new_dist
                nearest = node
        return nearest


    def _get_end_direction(self, node, borders, agent):

        nearest = None
        line_string = None
        dist = inf
        for line in borders:
            np = nearest_points(line, Point(node.x, node.y))
            curr_dist = ((node.x - np[0].x) ** 2 + (node.y - np[0].y) ** 2) ** .5
            if curr_dist < dist:
                nearest = np[0]
                line_string = line
                dist = curr_dist
        #
        # fig = plt.figure()
        # ax = plt.subplot(111)
        # ax.plot(nearest.x, nearest.y, 'bo')
        # ax.plot(node.x, node.y, 'rx')

        dist = inf
        second_pt = None
        dist_to_start = ((nearest.x - agent.position.x)** 2 + (nearest.y - agent.position.y) ** 2) ** .5
        for pt in line_string.coords:
            new_dist = ((pt[0] - agent.position.x) ** 2 + (pt[1] - agent.position.y) ** 2) ** .5
            # ax.plot(pt[0], pt[1], 'ro')
            # plt.pause(.1)
            if new_dist > dist_to_start and new_dist < dist:
                second_pt = pt
        # ax.plot(second_pt[0], second_pt[1], 'go')
        # plt.pause(.1)
        node.direction = atan2((second_pt[1] - nearest.y), (second_pt[0] - nearest.x))
        print("test")



class DubinsEdge:
    """
    Edge class to be used in path finding algorithms representing a dubins path
    """
    def __init__(self, start, end, dt=sp.TIME_STEP, time_int=0, agent_curvature=sp.MIN_CURVATURE * .25,
                 velocity=np.array([8.33, 0]), accelerate=False):
        """
        Dubins Edge Constructor

        :param start: start node of the edge
        :type start: Node
        :param end: desired end node of the edge
        :type end: Node
        :param dt: time increment
        :type dt: float
        :param time_int: time interval in which the edge is evaluated
        :type time_int: float
        :param is_complete: creates a complete edge from start to end, ignoring the time interval if True
        :type is_complete: bool
        :param agent_curvature: maximum curvature of the agent moving along the edge
        :type agent_curvature: float
        """
        self.origin = start
        self.dest = end
        self.length = 0
        self.curv = agent_curvature
        # create dubins path from start to end
        self.complete_x, self.complete_y, self.complete_yaw, self.mode, clen, self.lengths = \
            DubinsCurves.dubins_path_planning(Node(start.x, start.y, direction=start.direction), Node(end.x, end.y,
                                              direction=end.direction), self.curv)
        # initialize segment members
        self.tau = time_int
        self.segment = [0, 0, 0]
        self.linestring_segment = []
        self.segment_lengths = []

        # additional members to create part of dubins path
        self.dt = dt
        self.time = 0
        self.current_pos = np.array((1, 2), dtype=float)
        self.current_angle = 0
        self.velocity = velocity

        # create segment
        self.get_evaluation_pts()
        self.accelerate = accelerate
        if self.tau != 0:
            self.get_segment()
        self.calc_length()
        self.as_linestring = None
        self.is_invalid = False



    def get_segment(self):

        l_string = self.get_edge_as_linestring(False)
        if l_string is None:
            self.is_invalid = True
            return None
        trajectory = []
        directions = []
        speeds = []
        speed = self.velocity[0]
        time = 0
        distance_travelled = 0
        while distance_travelled < l_string.length and time < self.tau:
            trajectory.append(l_string.interpolate(distance_travelled))
            if self.accelerate:
                speed += sp.AVERAGE_ACCEL * self.dt
                if speed > sp.MAX_SPEED:
                    speed = sp.MAX_SPEED

            distance_travelled += speed * self.dt
            next_step = l_string.interpolate(distance_travelled)
            yaw = np.arctan2((next_step.y - trajectory[-1].y), (next_step.x - trajectory[-1].x))
            directions.append(yaw)
            speeds.append(speed)
            time += self.dt

        count = 0
        self.segment = [[], [], []]
        self.linestring_segment = LineString()
        for pt in trajectory:
            self.segment[0].append(pt.x)
            self.segment[1].append(pt.y)
            self.linestring_segment.union(pt)

        for yaw in directions:
            self.segment[2].append(yaw)

        self.dest = Node(self.segment[0][-1], self.segment[1][-1], self.segment[2][-1])
        if len(self.segment[0]) > 2:
            self.linestring_segment = LineString(trajectory)

        return trajectory, directions, speed

    def get_evaluation_pts(self):
        """
        Calculates the evaluation points of the edge segment from start to target, taking into account the time interval

        :param velocity: movement velocity of the agent given in metres per second
        :type velocity: Vector2
        :param is_complete: evaluates the complete edge from start to end if True, ignoring the time interval
        :type is_complete: bool
        """
        px, py, yaw = [], [], []
        px.append(self.origin.x)
        py.append(self.origin.y)
        yaw.append(self.origin.direction)
        self.current_pos = np.array([self.origin.x, self.origin.y])
        self.current_angle = self.origin.direction
        cnt = 0
        for i in range(0, 3):
            primitive = self.mode[i]
            length = self.lengths[i]
            if primitive == "S":
                px_s, py_s, yaw_s = self.get_straight_segment(length)
            elif primitive == "R":  # steering, turning_radius < 0
                px_s, py_s, yaw_s = self.get_turn_segment(length, True)
            elif primitive == "L":  # steering, turning_radius > 0
                px_s, py_s, yaw_s = self.get_turn_segment(length, False)
            px.extend(px_s)
            py.extend(py_s)
            yaw.extend(yaw_s)
        self.segment = [px, py, yaw]

    def get_straight_segment(self, length):
        """
        Calculates the evaluation points along a straight edge segment

        :param velocity: movement velocity of the agent moving along the segment given in metres per second
        :type velocity: Vector2
        :param length: total length of the straight segment given in m^-1
        :type length: float
        :param is_complete: evaluates the complete edge from start to end if True, ignoring the time interval
        :type is_complete: bool
        :return: returns the evaluation points including the resulting angles
        :rtype: (list[float], list[float], list[float])
        """
        dist_along_primitive = 0
        px, py, yaw = [], [], []
        length = length / self.curv
        while dist_along_primitive < length:
            new_pos = self.current_pos + hf.rotate((0, 0), self.velocity, self.current_angle) * self.dt
            dist_along_primitive += ((new_pos[1] - self.current_pos[1]) ** 2 + (new_pos[0] - self.current_pos[0]) ** 2) ** .5

            if dist_along_primitive > length:
                delta = ((new_pos[1] - self.current_pos[1]) ** 2 + (new_pos[0] - self.current_pos[0]) ** 2) ** .5 - \
                        (dist_along_primitive - length)
                dist_along_primitive -= ((new_pos[1] - self.current_pos[1]) ** 2 + (
                            new_pos[0] - self.current_pos[0]) ** 2) ** .5
                dt_2 = self.dt * delta / ((
                            (new_pos[1] - self.current_pos[1]) ** 2 + (new_pos[0] - self.current_pos[0]) ** 2) ** .5)
                new_pos = self.current_pos + hf.rotate((0, 0), self.velocity, self.current_angle) * dt_2
                self.time += dt_2
                dist_along_primitive += ((new_pos[1] - self.current_pos[1]) ** 2 + (
                            new_pos[0] - self.current_pos[0]) ** 2) ** .5

                self.current_pos = new_pos
                px.append(new_pos[0])
                py.append(new_pos[1])
                yaw.append(self.current_angle)
                break
            self.current_pos = new_pos
            px.append(new_pos[0])
            py.append(new_pos[1])
            yaw.append(self.current_angle)
        self.segment_lengths.append(dist_along_primitive)
        return px, py, yaw

    def get_turn_segment(self, turn_angle, right=True): # turn_angle ist auch im Bogenmaß
        """
        Calculates the evaluation points along a turning edge segment

        :param velocity: movement velocity of the agent moving along the segment given in metres per second
        :type velocity: Vector2
        :param turn_angle: turning angle of the segment given in radians/m
        :type turn_angle: float
        :param right: True if the segment is a right turn
        :type right: bool
        :param is_complete: evaluates the complete edge from start to end if True, ignoring the time interval
        :type is_complete: bool
        :return: returns the evaluation points including the resulting angles
        :rtype: (list[float], list[float], list[float])
        """
        if right:
            turning_r = -self.curv ** (- 1)
        else:
            turning_r =  self.curv ** (- 1)

        turned_angle = 0 # Bogenmaß
        # current_angle ist in Grad
        px, py, yaw = [], [], []
        angular_velocity = self.velocity[0] / turning_r  # Bogenmaß

        while turned_angle < turn_angle:
            new_pos_x = self.current_pos[0] - turning_r * sin(self.current_angle) + \
                        turning_r * sin(angular_velocity * self.dt + self.current_angle)
            new_pos_y = self.current_pos[1] + turning_r * cos(self.current_angle) - \
                        turning_r * cos(angular_velocity * self.dt + self.current_angle)
            new_angle = angular_velocity * self.dt + self.current_angle
            new_pos = np.array([new_pos_x, new_pos_y])
            turned_angle += abs(angular_velocity * self.dt)

            if turned_angle > turn_angle:
                delta = abs(angular_velocity * self.dt) - (turned_angle - turn_angle)
                dt_2 = abs(delta / angular_velocity)
                turned_angle -= abs(angular_velocity * self.dt)

                new_pos_x = self.current_pos[0] - turning_r * sin(self.current_angle) + \
                            turning_r * sin(angular_velocity * dt_2 + self.current_angle)
                new_pos_y = self.current_pos[1] + turning_r * cos(self.current_angle) - \
                            turning_r * cos(angular_velocity * dt_2 + self.current_angle)
                new_pos = np.array([new_pos_x, new_pos_y])
                new_angle = angular_velocity * dt_2 + self.current_angle

                turned_angle += abs(angular_velocity * dt_2)
                self.current_pos = new_pos
                self.current_angle = new_angle
                px.append(new_pos[0])
                py.append(new_pos[1])
                yaw.append(self.current_angle)
                break
            else:
                self.current_pos = new_pos
                self.current_angle = new_angle
            px.append(new_pos[0])
            py.append(new_pos[1])
            yaw.append(self.current_angle)
        self.segment_lengths.append(turned_angle / sp.MIN_CURVATURE)
        return px, py, yaw

    def calc_length(self):
        """
        Calculates the length of the complete edge
        """
        for el in self.segment_lengths:
            self.length += el

    def get_edge_as_linestring(self, simplified=False):
        """
        Returns the current Dubins Edge as an LineString object

        :return: the respective LineString
        :rtype: LineString
        """
        points = []
        if simplified:
            points.append([self.segment[0][0], self.segment[1][0]])
            points.append([self.segment[0][-1], self.segment[1][-1]])
        else:
            for i in range (0, len(self.segment[0])):
                points.append([self.segment[0][i], self.segment[1][i]])

        if len(self.segment[0]) < 2:
            return None
        else:
            self.as_linestring = LineString(points)
            return LineString(points)

    def plot(self, ax, style='k-'):
        """
        Plots the current Dubins Edge onto a given plot data series

        :param ax: given plot data series
        :type ax: plt.Axes
        :param style: the linestyle of the edge
        :type style: str
        """
        ax.plot(self.segment[0], self.segment[1], style)


class DubinsTree:
    """
    Tree Class for path finding algorithms facilitating dubins paths
    """
    def __init__(self, start=None, goal=None, rad=None, time_int=100, ax=None):
        """
        Dubins Tree Constructor

        :param start: start node
        :type start: Node
        :param goal: goal node of the tree, which will be tried to reach in a path planning problem
        :type goal: Node
        :param rad: radius around the goal node, if a node reaches the resulting circle, the goal counts as reached
        :type rad: float
        :param time_int: time interval in which the dubins paths are evaluated given in seconds
        :type time_int: float
        :param ax: plot data series in case of plotting
        :type ax: Axes
        """
        self.goal_node = goal
        self.start_node = start
        self.goal_radius = rad
        self.nodes = []
        self.beacons = []
        self.dubins_edges = []
        self.rewired = False
        self.time_intervall = time_int
        if start:
            self.insert_node(start, ax)
        if goal and ax:
            self.goal_node.plot(color.tree_start_nd, ax, r=rad)

    def insert_node(self, node, ax):
        """
        Adds a new node to the tree

        :param node: the new node to be added
        :type node: Node
        :param ax: the plot data series in case of plotting
        :type ax: Axes
        :return: if plotted the new node plot patch is returned
        :rtype: patches.Circle
        """
        self.nodes.append(node)
        if node.type == 'start':
            clr = color.tree_start_nd
        elif node.type == 'nearest':
            clr = color.tree_nearest_nd
        elif node.type == 'new':
            clr = color.tree_new_nd
        else:
            clr = color.tree_nd

        if ax:
            new_node = node.plot(clr, ax)
            return new_node

    def insert_dubins_path(self, path):
        self.dubins_edges.append(path)

    def draw_tree(self, ax, new_node=None):
        """
        Draws the tree to a given plot data series

        :param ax: the plot data series
        :type ax: Axes
        :param new_node: If a new node is given, only the new edge is plotted
        :type new_node: Node
        """
        if self.rewired or not new_node:
            for i in range(0, len(ax.lines)):
                ax.lines.pop(-1)
            for node in self.nodes:
                if node.parent:
                    ax.plot(node.path[0], node.path[1], '-', linewidth='.7')
        else:
            ax.plot(new_node.path[0], new_node.path[1], '-', linewidth='.7')

    def draw_nodes(self, ax, style=None):
        """
        Plots the nodes in the tree into a given plot data series

        :param ax: plot data series
        :type ax: Axes
        :param style: marker style of the nodes. If None, the default node plot style is used
        :type style: str
        """
        for node in self.nodes:
            if not style:
                node.plot(color.tree_nd, ax)
            else:
                ax.plot(node.x, node.y, style)

    def plot_target_path(self, successful_node, ax):
        """
        Plots the path from the start to the target node

        :param successful_node: end node of the plotted path, usually the node within the goal circle
        :type successful_node: Node
        :param ax: plot data series
        :type ax: Axes
        """
        print("Total cost: ", successful_node.cost)
        while successful_node.parent:
            ax.plot(successful_node.path[0], successful_node.path[1], color=color.tree_nearest_nd, linewidth='1')
            successful_node = successful_node.parent

    def plot_beacons(self, ax, radius=12):
        """
        Plots the beacons to the given plot data series

        :param radius: the radius of the beacon circle to the be drawn
        :type radius: float
        :param ax: plot data series
        :type ax: Axes
        """
        for node in self.beacons:
            beacon_patch = matplotlib.patches.Circle((node.x, node.y), radius=radius, facecolor=color.tree_new_nd, alpha=.5,
                                      edgecolor=color.tree_new_nd)
            ax.add_patch(beacon_patch)

    def find_node_closest_to_goal(self):
        """
        Finds the node in the tree, that is closest to the goal node. The direct distance is used and not the length
        of the dubins edge.

        :return: closest tree node to goal
        :rtype: Node
        """
        current_dist = inf
        nearest = Node()
        for node in self.nodes:
            new_dist = calc_node_dist(node, self.goal_node)
            if new_dist < current_dist:
                current_dist = new_dist
                nearest = node
        return nearest

    def get_node_by_nbr(self, nbr):
        """
        Gets a not with the given number in the tree, if exist

        :param nbr: number of the node to be found
        :type nbr: int
        :return: return the node wit the given number
        :rtype: Node
        """
        for node in self.nodes:
            if node.nbr == nbr:
                return node
        return None

    def plot(self, ax, new_node=None):
        """
        Draws the tree to a given plot data series

        :param ax: the plot data series
        :type ax: Axes
        :param new_node: If a new node is given, only the new edge is plotted
        :type new_node: Node
        """
        if self.rewired or not new_node:
            for i in range(0, len(ax.lines)):
                ax.lines.pop(-1)
            for node in self.nodes:
                if node.parent:
                    ax.plot([node.parent.x, node.x], [node.parent.y, node.y], '-', linewidth='.7')
        else:
            ax.plot([new_node.parent.x, new_node.x], [new_node.parent.y, new_node.y], '-', linewidth='.7')


class Agent:
    """
    Agent class storing all the parameters of the agent moving in the world

    :param position: the agent's current position
    :type position: Node
    :param global_position: the agent's position in sumo global coordinates
    :type global_position: tuple[float, float]
    :param angle: the agent's current angle given in degrees from the east
    :type angle: float
    :param length: length of the agent, taking into account scaling
    :type length: float
    :param width: width of the agent, taking into account scaling
    :type width: float
    :param max_curvature: the maximum curvature of the agent's turning moves
    :type max_curvature: float
    :param conflict_vehicles: the agent'S current conflicting vehicles
    :type conflict_vehicles: list[str]
    :param type: the type of the agent (i.e ego-vehicle)
    :type type: str
    :param id: the ID of the agent used in SUMO
    :type id: str
    :param sumo_controlled: specifies, whether the agent is autonomous or controlled by SUMO
    :type sumo_controlled: bool
    :param trajectory: the agent's anticipated trajectory
    :type trajectory: np.ndarray
    :param speed: the agent's current speed
    :type speed: np.array
    :param colour: colour used for plots
    :type colour: tuple[float, float, float]
    :param sumo_colour: colour used within SUMO
    :type sumo_colour: tuple[float, float, float]
    :param route_id: dummy parameter used for the agent'S current route
    :type route_id: str
    :param controller: the agent's controller
    :type controller: control.Controller
    :param destination: the agent's anticipated destination
    :type destination: Node
    :param senssors: the agent's fitted sensors
    :type sensors: list[Sensor]

    """
    def __init__(self, start=Node(0, 0, 0), length=sp.CAR_LENGTH, width=sp.CAR_WIDTH, c=sp.MIN_CURVATURE,
                 type="TestCar", autonomous=True, sumo_data=None, speed=sp.START_SPEED):
        """
        Agent Class Constructor

        :param speed: the agent's starting speed
        :type speed: np.array
        :param type: the type of the agent (i.e ego-vehicle)
        :type type: str
        :param autonomous: if True the agent is not controlled by a sumo recorded trajectory
        :type autonomous: bool
        :param sumo_data: the agent's data collected from sumo
        :type sumo_data: np.ndarray
        :param start: start position of the agent
        :type start: Node
        :param length: length of the agent, taking into account scaling
        :type length: float
        :param width: width of the agent, taking into account scaling
        :type width: float
        :param c: maximum possible curvature of the agent based on the turning radius
        :type c: float
        """
        # START VALUES
        self.start_pos = copy.deepcopy(start)
        self.start_angle = start.direction
        self.start_speed = speed
        self.start_trajectory = copy.deepcopy(sumo_data)

        self.position = start
        self.global_position = None
        self.angle = start.direction
        self.length = length
        self.width = width
        self.max_curvature = c
        self.conflict_vehicles = []
        self.type = type
        self.id = str()
        self.sumo_controlled = not autonomous
        self.trajectory = None
        self.speed = np.array([speed, 0], dtype=float)
        if not autonomous:
            self.sumo_controlled = not autonomous
            self.make_sumo_agent(sumo_data)
        self.colour, self.sumo_colour = self._set_colour()
        self.route_id = "route_1"
        self.sensors = []
        self.controller = None
        self.destination = None
        self.trajectory_step = 0
        self.field_of_vision = None
        self.has_stopped = False
        self.distance_travelled = None
        self._recorded_trajectory = [[],[],[],[],[]]
        self.recorded_trajectory = None
        self.cfm = None

    def reset(self):
        """
        Resets the agent to its start position.
        """
        self.position = copy.copy(self.start_pos)
        self.angle = self.start_angle
        self.speed[0] = self.start_speed
        self.trajectory = copy.copy(self.start_trajectory)
        self.trajectory_step = 0
        self.recorded_trajectory = None
        self._recorded_trajectory = [[],[],[],[],[]]
        self.distance_travelled = None

    def decelerate(self, dec_rate=sp.FREE_DECEL):
        """
        Decelerates the agent by the given rate.

        :param dec_rate: the deceleration
        :type dec_rate: float
        """
        self.set_speed(self.speed[0] - dec_rate * sp.TIME_STEP)

    def get_speed(self):
        """
        Returns the agent's current speed

        :return: the agent's current speed
        :rtype: np.array
        """
        return self.speed

    def set_speed(self, v=None):
        """
        Sets the agent's speed to either the chosen value or accelerates taking into account the average deceleration

        :param v: the anticipated speed
        :type v: float
        """
        if v is not None:
            new_speed = v
        else:
            new_speed = self.speed[0] + sp.AVERAGE_ACCEL * sp.TIME_STEP

        if new_speed > sp.MAX_SPEED:
            new_speed = sp.MAX_SPEED
        self.speed[0] = new_speed

    def get_stopping_distance(self, dt=sp.TIME_STEP, emergency=False):
        """
        Calculates the stopping distance of the agent.

        :param dt: the length of the sumo time step
        :type dt: float
        :param emergency: if True an emergency braking is made using the maximum brake deceleration
        :type emergency: bool
        :return: the stopping distance
        :rtype: float
        """
        decel = sp.MAX_BRAKE_DECEL if emergency else sp.COMFORT_BRAKE_DECEL
        stopping_dist = 0
        current_speed = self.get_speed()[0]
        while current_speed > 0:
            current_speed -= decel * dt
            stopping_dist += current_speed * dt
        return stopping_dist

    def set_destination(self, dest):
        """
        Sets the desired destination of the agent and assigns it to the controller

        :param dest: the anticipated destination
        :type dest: Node
        """
        self.destination = dest
        if self.controller:
            self.controller.destination = self.destination

    def add_sensor(self, sensor):
        """
        Adds a sensor to the current agent.

        :param sensor: the sensor to be added
        :type sensor: Sensor
        """
        sensor.initialize()
        self.sensors.append(sensor)

    def do_sensor_update(self, borders, max_dist):
        """
        Updates the agent's sensors

        :param borders: the borders of the current road
        :type borders: list[LineString]
        :param max_dist: the maximum distance in which the agent can recognise obstacles
        :type max_dist: float
        :return: returns the data gatheres by the sensors
        :rtype: tuple[list[np.array], list[np.array]]
        """
        car_data, border_data = [], []
        for sensor in self.sensors:
            # self.get_conflict_agents(max_dist)
            new_border_data, new_car_data = sensor.update_sensor(borders)
            car_data.extend(new_car_data)
            border_data.extend(new_border_data)
            # else:
            #     sensor.update_sensor()
        return  border_data, car_data

    def move_to_pos(self, new_pos, ax=None):
        """
        Moves the agent to the given position

        :param new_pos: the desired position of the agent
        :type new_pos: Node
        """
        self.position = new_pos
        self.angle = new_pos.direction
        if ax:
            return self.plot_agent(ax)

    def get_global_position(self):
        """

        :param pos:
        :type pos:
        """
        self.global_position = traci.vehicle.getPosition(self.id)
        return self.global_position

    def move_along_path(self, edge=None, end_node=None, ax=None):
        """
        Moves the agent along a path

        :param edge: the path along which the agent is to be moved gives as a DubinsEdge
        :type edge: DubinsEdge.segment
        :param end_node: if an end node is given the agent moves to the end node along the path to it
        :type end_node: Node
        :param ax: plot data series. If given, the agents bounding box will be plotted
        :type ax: Axes
        """
        if end_node:
            complete_x = []
            complete_y = []
            complete_yaw = []
            while end_node.parent:
                for i in range(0, len(end_node.path)):
                   end_node.path[i].reverse()
                complete_x.extend(end_node.path[0])
                complete_y.extend(end_node.path[1])
                complete_yaw.extend(end_node.path[2])
                end_node = end_node.parent
                edge = [complete_x, complete_y, complete_yaw]
                for i in range(0, len(edge)):
                    edge[i].reverse()

        i = 0
        current_rect = None
        for i in range(0, len(edge[0])):
            if ax and current_rect:
                ax.patches.remove(current_rect)
            self.move_to_pos(Node(edge[0][i], edge[1][i], edge[2][i]))
            if ax:
                current_rect = self.plot(ax)
            i += 1

    def get_conflict_agents(self, max_dist):
        """
        Gets all the vehicles in the current network in a given range

        :param max_dist: the range around the agent in which other vehicles will be considered
        :type max_dist: float
        :return: the id-list list of the vehicles in the given range
        """
        vehicle_list = traci.vehicle.getIDList()
        conflicting_vehicles = dict()

        for id in vehicle_list:
            if traci.vehicle.getLaneID(id) == traci.vehicle.getLaneID(self.id):
                veh_pos = traci.vehicle.getPosition(id)
                if str.__eq__(self.id, id):
                    self.get_global_position()
                else:
                    conflicting_vehicles[id]= veh_pos

        distances = dict()
        for i,v in enumerate(conflicting_vehicles.values()):
            distances[i] = ((v[0] - self.global_position[0]) ** 2 + (v[1] - self.global_position[1]) ** 2) ** .5

        conflict_vehicles_copy = copy.copy(conflicting_vehicles)
        for j,k in zip(conflict_vehicles_copy.keys(), distances.keys()):
            if distances[k] > max_dist:
                del conflicting_vehicles[j]

        self.conflict_vehicles = [v for (d, v) in sorted(zip(distances, conflicting_vehicles), key=lambda p: p[0])]
        shf.recolor_agent_list(self.conflict_vehicles, sp.COLOURS.conflicting_vehicle)
        return [v for (d, v) in sorted(zip(distances, conflicting_vehicles), key=lambda p: p[0])] # sort by distance

    def make_sumo_agent(self, rider_data):
        """
        Creates an non-autonomous agent controlled by data collected from sumo

        :param rider_data: the sumo collected data
        :type rider_data: np.ndarray
        """
        self.trajectory = rider_data
        self.set_start_time_to_value()
        self.interpolate_trajectory()
        self.get_distance_travelled()

    def interpolate_trajectory(self, step_length=sp.TIME_STEP):
        """
        Interpolates the values of the tum simulation to the given time step.

        :param step_length: the time step length
        :type step_length: float
        """
        nbr_of_steps = int(self.trajectory[-1, 0] / step_length)
        interpolated_trajectory = np.ndarray((nbr_of_steps, 6))

        time = 0.0
        interpolated_trajectory[0, :] = self.trajectory[0, :]
        interpolated_trajectory[0, 1] = 0.0
        for i in range(0, nbr_of_steps - 1):
            time += step_length
            j = self._find_higher_values(time)
            interpol_values = self._interpolate(interpolated_trajectory[i, :], time, j, step_length)
            interpolated_trajectory[i + 1, :] = interpol_values

        self.trajectory = interpolated_trajectory
    
    def _find_higher_values(self, current_time_step):
        """
        Finds the next higher time value within the trajectory according to the given current time.

        :param current_time_step: the value to start from
        :type current_time_step: float
        :return: the index of the next higher value within self.trajectory
        :rtype: int
        """
        for i in range (0, self.trajectory.shape[0]):
            if self.trajectory[i, 0] > current_time_step:
                return i

    def _interpolate(self, last_values, current_time, i, time_step):
        """

        :param current_time: the current time step for interpolation
        :type current_time: float
        :param i: The row containing the value higher than the current time_step
        :type i: int
        """
        pos_x = last_values[2] + (self.trajectory[i, 2] - last_values[2]) / (
                self.trajectory[i, 0] - last_values[0]) * time_step
        pos_y = last_values[3] + (self.trajectory[i, 3] - last_values[3]) / (
                    self.trajectory[i, 0] - last_values[0]) * time_step
        speed = last_values[4] + (self.trajectory[i, 4] - last_values[4]) / (
                self.trajectory[i, 0] - last_values[0]) * time_step
        dir = self.trajectory[i, 5] #+ (self.trajectory[i, 5] - last_values[5]) / (
        #         self.trajectory[i, 0] - last_values[0]) * time_step
        return np.array([current_time, current_time, pos_x, pos_y, speed, dir])

    def plot_trajectory(self, ax, style='r-'):
        """
        Plots the agent's trajectory to a given plot data series

        :param ax: the plot data seris
        :type ax: plt.Axes
        :param style: line style of the trajectory
        :type style: str
        """
        ax.plot(self.trajectory[:,2], self.trajectory[:, 3], style)

    def set_start_time_to_value(self, start_time=None):
        """
        Sets the start time of the agent to a given start time

        :param start_time: start-time to be used
        :type start_time: float
        """
        if start_time:
            print("implement later")
        else:
            start_val = self.trajectory[0][0]
            for i in range(0, self.trajectory.shape[0]):
                self.trajectory[i][0] -= start_val

    def update_trajectory(self, step=-1):
        """
        Updates the agents current values to the given time-step

        :param step: the given time step
        :type step: int
        """
        if step >=0:
            if step < self.trajectory.shape[0] - 1:

                self.position.x = self.trajectory[step, 2]
                self.position.y = self.trajectory[step, 3]
                self.angle = radians(shf.get_plt_angle(self.trajectory[step, 5]))
                self.position.direction = radians(self.angle)
                self.speed[0] = self.trajectory[step, 4]
        else:
            self.position.x = traci.vehicle.getPosition(self.id)[0]
            self.position.y = traci.vehicle.getPosition(self.id)[1]
            self.angle = shf.get_plt_angle(traci.vehicle.getAngle(self.id))
            self.speed[0] = traci.vehicle.getSpeed(self.id)

    def record_trajectory(self):
        if len(self._recorded_trajectory[0]) == 0:
            self._recorded_trajectory[0].append(0)
        else:
            self._recorded_trajectory[0].append(self._recorded_trajectory[0][-1] + sp.TIME_STEP)
        self._recorded_trajectory[1].append(self.position.x)
        self._recorded_trajectory[2].append(self.position.y)
        self._recorded_trajectory[3].append(self.speed[0])
        self._recorded_trajectory[4].append(shf.get_sumo_angle(self.angle))

    def _change_recorded_trajectory_to_ndarray(self):
        self.recorded_trajectory = np.ndarray((len(self._recorded_trajectory[0]), 6), dtype=float)
        self.recorded_trajectory[:, 0] = np.asarray(self._recorded_trajectory[0])
        self.recorded_trajectory[:, 1] = np.asarray(self._recorded_trajectory[0])
        self.recorded_trajectory[:, 2] = np.asarray(self._recorded_trajectory[1])
        self.recorded_trajectory[:, 3] = np.asarray(self._recorded_trajectory[2])
        self.recorded_trajectory[:, 4] = np.asarray(self._recorded_trajectory[3])
        self.recorded_trajectory[:, 5] = np.asarray(self._recorded_trajectory[4])

        del self.trajectory
        self.trajectory = self.recorded_trajectory

    def update(self, pos_x, pos_y, angle, speed=8.33, conflict_veh=None, path_end=None):
        """
        Updates the agents current values

        :param step: the given time step
        :type step: int
        """
        self.position.x = pos_x
        self.position.y = pos_y
        self.angle = angle
        self.position.direction = self.angle
        self.speed[0] = speed
        if conflict_veh:
            crit_pos = traci.vehicle.getPosition(conflict_veh)
            crit_dist = ((crit_pos[0] - self.get_global_position()[0]) ** 2 +
                         (crit_pos[1] - self.get_global_position()[1]) ** 2) ** .5
            if path_end:
                dist_to_path_end = calc_node_dist(self.position, path_end)
            return crit_dist, dist_to_path_end

    def plot(self, ax, include_sensor=sp.PLOT_SENSOR, plot_cone=sp.PLOT_CONE, include_vision_field=sp.PLOT_FIELD_OF_VIEW,
             fancy=True):
        """
        Plots the agent as a rectangle to a given plot data seris

        :param ax: the given plot data series
        :type ax: plt.Axes
        :return: returns the plotted rectangle
        :rtype: patches.Rectangle
        """
        image = plt.imread(params.IMG_PATHS().agent)
        img = ax.imshow(image, origin="lower", zorder=10)

        scale = sp.CAR_WIDTH / image.shape[0]
        tscale = matplotlib.transforms.Affine2D().scale(scale)

        tx1 = self.length / 2
        ty1 = self.width / 2
        ttranslate1 = matplotlib.transforms.Affine2D().translate(-tx1, -ty1)

        tr = matplotlib.transforms.Affine2D().rotate_deg(degrees(self.angle))

        tx = self.position.x
        ty = self.position.y
        ttranslate2 = matplotlib.transforms.Affine2D().translate(tx, ty)

        trans_data = tscale + ttranslate1 + tr + ttranslate2 + ax.transData
        img.set_transform(trans_data)
        # image = plt.imread(params.IMG_PATHS().agent)

        # ts = ax.transData
        # coords = [self.position.x, self.position.y]
        # tr = matplotlib.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], degrees(self.angle))
        # t = tr + ts
        # img = ax.imshow(image, origin="lower", transform=t)
        # # img.set_transform(t)


        # ts = ax.transData
        # coords = [self.position.x, self.position.y]
        # tr = matplotlib.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], degrees(self.angle))
        # t = tr + ts
        #
        # rect = patches.Rectangle((self.position.x - self.length / 2, self.position.y -
        #                           self.width / 2), self.length, self.width, linewidth=1, color=self.colour,
        #                          transform=t)
        # rect.set_transform(t)
        # ax.add_patch(rect)


        if include_sensor:
            for sensor in self.sensors:
                if plot_cone:
                    sensor.plot_cone(ax)
                else:
                    sensor.plot(ax)

        if include_vision_field:
            if fancy:
                splt.plot_field_of_view(self.field_of_vision, ax)
            else:
                self.field_of_vision.plot(ax)

        return img

    def _set_colour(self):
        """
        Sets the colours for plotting and sumo

        :return: the sumo and normal plot colour
        :rtype: tuple[tuple, tuple]
        """
        clr = color.mpl_line_green
        sumo_clr = sp.COLOURS.ego_vehicle
        if self.type == "ego":
            clr = color.mpl_line_red
            sumo_clr = color.mint
        elif self.type == "flow":
            clr = color.mpl_line_blue
            sumo_clr = color.light_blue
        elif self.type == "conflict":
            clr = color.mpl_line_purple
            sumo_clr = color.light_green
        elif self.type == "bike":
            sumo_clr = sp.AgentColors().bike
        return clr, sumo_clr

    def sumo_move(self, edge="P_in"):
        """
        Moves the agent to its current position within a sumo simulation

        :param edge: just a random edge (does not play a role since I am using moveToXY)
        :type edge: str
        """
        traci.vehicle.moveToXY(vehID=self.id, edgeID=edge, lane=0, x=self.position.x, y=self.position.y,
                                                      angle=shf.get_sumo_angle(self.angle), keepRoute=2)

    def add_to_sumo(self, agent_id, route_id="route1"):
        """
        Adds the autonomous agent to the current SUMO simulation.

        :param agent_id: the ID of the agent to be added
        :type agent_id: str
        :param route_id: the ID of the route to be used
        :type route_id: str
        """
        self.id = agent_id
        shf.spawn_agent_at_position(self, route_id)
        traci.vehicle.setSpeedMode(self.id, 0)
        self.get_global_position()

    def print_trajectory_values(self):
        if self.trajectory.any():
            acc_sum = 0
            v_sum = 0
            max_v = 0
            max_a = 0
            for i in range(0, len(self.trajectory) - 1):
                current_v = self.trajectory[i, 4]
                current_a = (self.trajectory[i + 1, 4] - self.trajectory[i, 4]) / \
                            (self.trajectory[i + 1, 0] - self.trajectory[i, 0])
                if current_v > max_v:
                    max_v = current_v
                if current_a > max_a:
                    max_a = current_a

                acc_sum += current_a
                v_sum += current_v

            v_sum += self.trajectory[-1, 4]

            avrg_v = v_sum / len(self.trajectory)
            avrg_a = acc_sum / len(self.trajectory)

            print("Starting Position: " + "(" + str(self.trajectory[0, 2]) + "," + str(self.trajectory[0, 3]))
            print("Start Speed: " + str(self.trajectory[0, 4]))
            print("Start Orientation: " + str(shf.get_plt_angle(self.trajectory[0, 5])))
            print("Average Speed: " + str(avrg_v))
            print("Average Acceleration: " + str(avrg_a))
            print("Maximum Speed: " + str(max_v))
            print("Maximum Acceleration: " + str(max_a))
            print("Maximum Braking: " + str(min(self.get_acceleration())))

    def get_distance_travelled(self):
        self.distance_travelled = np.zeros((self.trajectory.shape[0], 1))
        self.distance_travelled[0] = 0
        for i in range(1, len(self.trajectory)):
            new_dist = ((self.trajectory[i - 1, 2] - self.trajectory[i, 2]) ** 2 + (self.trajectory[i - 1, 3] - self.trajectory[i, 3])** 2) ** .5
            self.distance_travelled[i] = self.distance_travelled[i - 1] + new_dist
        return self.distance_travelled

    def get_acceleration(self):
        accelerations = np.zeros(self.trajectory.shape[0])
        for i in range(0, self.trajectory.shape[0] - 1):
            accelerations[i] = (self.trajectory[i + 1, 4] - self.trajectory[i, 4]) / \
                        (sp.TIME_STEP)
        accelerations[-1] = accelerations[-2]
        return accelerations

    # Plotting Functions
    def plot_dist_time(self, ax, color=None, label="", style='solid'):
        times = self.trajectory[:, 0]
        ax.plot(times, self.get_distance_travelled(), color=color, label=label, linestyle=style)

    def plot_dist_to_conflict(self, ax, distances, color=None, label="", style='solid'):
        ax.plot(self.trajectory[:, 0], distances, color=color, label=label, linestyle=style)

    def plot_speeds(self, ax, color=None, label="", style='solid'):
        ax.plot(self.trajectory[:, 0], self.trajectory[:, 4], color=color, label=label, linestyle=style)

    def plot_acceleration(self, ax, color=None, label="", style='solid'):
        ax.plot(self.trajectory[:, 0], self.get_acceleration(), color=color, label=label, linestyle=style)

    def safe_trajectory_to_csv(self, path):
        traj_data = pd.DataFrame({'time': self.trajectory[:, 0], 'sim_time': self.trajectory[:, 1],
                      'TESIS_0_x': self.trajectory[:, 2], 'TESIS_0_y': self.trajectory[:, 3],
                      'TESIS_0_speed': self.trajectory[:, 4], 'TESIS_0_angle': self.trajectory[:, 5],
                                  'Accel': self.get_acceleration()})

        traj_data.to_csv(path, index=False, header=True)


def plot_agent(pos, angle, ax, clr=color.mpl_line_blue, length=5, width=2.33):
    """
    Plots a single vehicle to a given plot data series

    :param pos: the vehicles position
    :type pos: tuple[float]
    :param angle: the vehicles angle given in [rad]
    :type angle: float
    :param ax: the plot data series
    :type ax: plt.Axes
    :param clr: the color of the plotted rectangle
    :type clr: hf.Colours
    :param length: The vehicles length
    :type length: float
    :param width: The vehicles width
    :type width: float
    :return: returns the plotted patch
    :rtype: patches.Rectangle
    """
    if ax:
        ts = ax.transData
        coords = [pos[0], pos[1]]
        tr = matplotlib.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], degrees(angle))
        t = tr + ts

        rect = patches.Rectangle((pos[0] - length / 2, pos[1] -
                                  width / 2), length, width, linewidth=1, color=clr,
                                 transform=t)
        rect.set_transform(t)
        ax.add_patch(rect)

        return rect

class Sensor:
    """
    This Class mimics the behaviour of a lidar sensor and/or a camera with object recognition.
    """
    def __init__(self, parent=None, reach=0, angle=0, cover=100, direction=0, pos=np.array([0, 0]), type=None):
        """
        Sensor Constructor

        :param parent: the Agent to which the sensor is attached
        :type parent: Agent
        :param reach: the reach of the sensor
        :type reach: float
        :param angle: the covering angle of the sensor in degrees
        :type angle: float
        :param cover: the coverage rate of the sensors cone in percent
        :type cover: int
        :param direction: the sensors direction in degrees
        :type direction: float
        :param pos: the position relative to the cars back right of sensor
        :type pos: np.array
        """
        self.parent = parent
        self.position = pos
        self.reach = reach
        self.angle = angle
        self.start_direction = direction
        self.direction = self.start_direction + degrees(self.parent.angle)
        self.coverage = cover
        self.origin = self.get_origin()
        self.frequency = 1
        self.detection_lines_list = []
        self.car_data = []
        self.border_data = []
        self.cone = None
        self.type = type
        self.rec_vehicles = False
        self.plot_lines = []

    def get_origin(self):
        """
        Calculates the the sensors origin in world coordinates based on the agent's current position.

        :return: the sensor's origin in world coordinates
        :rtype: np.array
        """
        x = self.position[0] - sp.CAR_LENGTH / 2 + self.parent.position.x
        y = self.position[1] - sp.CAR_WIDTH / 2 + self.parent.position.y

        parent_position = np.array([self.parent.position.x, self.parent.position.y])
        x, y = hf.rotate(parent_position, np.array([x, y]), self.parent.angle)

        return np.array([x, y])

    def initialize(self):
        """
        Initializes the sensor, creating all the rays.
        """
        self.detection_lines_list = []
        turning_angle = self.direction + self.angle / 2

        delta_phi = self.angle / (self.coverage + 1)
        for i in range(0, self.coverage + 2):
            x_i = self.origin[0] + self.reach * sin(radians(i * delta_phi - turning_angle + 90))
            y_i = self.origin[1] + self.reach * cos(radians(i * delta_phi - turning_angle + 90))
            ray = SensorRay(self, np.array([x_i, y_i]))
            self.detection_lines_list.append(ray)

    def update_sensor(self, borders):
        """
        Does an update for the sensor, calculating the new ray intersection points and the collected data.

        :return: returns the sensor collected border and car data
        :rtype: tuple[list[array], list[array]]
        :param borders: the borders of the current road
        :type borders: list[LineString]
        """
        self.border_data = []
        self.car_data = []

        self.direction = self.start_direction +  degrees(self.parent.angle)
        self.origin = self.get_origin()
        self.initialize()

        for ray in self.detection_lines_list:
            self.recognise_borders(ray, borders)
            if self.rec_vehicles:
                self.recognise_vehicles(ray)

            ray.choose_intersection()
            if ray.target_type == 0:
                self.border_data.append(ray.target)
            elif ray.target_type == 1:
                self.car_data.append(ray.target)

        return self.border_data, self.car_data

    def recognise_borders(self, ray, borders):
        """
        Detects the intersection points with te road borders for the given sensor ray.

        :param ray: the sensor ray to be evaluated
        :type ray: SensorRay
        :param borders: the road borders
        :type borders: list[LineString]
        """
        for border in borders:
            intersection_pt = ray.line.intersection(border)
            if isinstance(intersection_pt, MultiPoint):
                for pt in intersection_pt:
                    ray.temp_inter.append([(pt.x, pt.y), 0])
            elif intersection_pt:
                ray.temp_inter.append([intersection_pt.coords[0], 0])

    def recognise_vehicles(self, ray):
        """
        Detects the intersection points with te road borders for the given sensor ray.

        :param ray: the sensor ray to be evaluated
        :type ray: SensorRay

        """
        for veh in self.parent.conflict_vehicles:
            veh_rect = shf.get_sumo_vehicle_as_poly(veh)
            intersection_pt = ray.line.intersection(veh_rect)
            if isinstance(intersection_pt, MultiPoint):
                for pt in intersection_pt:
                    ray.temp_inter.append([(pt.x, pt.y), 0])
            elif intersection_pt:
                ray.temp_inter.append([intersection_pt.coords[0], 1])

    def plot(self, ax, style='r-'):
        """
        Plots the current sensor to a given plot data series.

        :param ax: the given plot data series
        :type ax: plt.Axes
        :param style: the plot style of the sensor rays
        :type style: str
        """
        self._delete_old(ax)
        for ray in self.detection_lines_list:
            line = ray.plot(ax, style)
            self.plot_lines.append(line)

    def plot_data(self, ax):
        """
        Plots the data collected by the sensor.

        :param ax: the plot data series
        :type ax: plt.Axes
        """
        for pt in self.car_data:
            ax.plot(pt[0], pt[1], 'kx')
        for pt in self.border_data:
            ax.plot(pt[0], pt[1], 'rx')

    def make_cone(self):
        """
        Creates a cone for the current sensor

        """
        point_list = [(self.origin[0], self.origin[1])]
        for ray in self.detection_lines_list:
            point_list.append((ray.target[0], ray.target[1]))
        point_list.append((self.origin[0], self.origin[1]))

        self.cone = Polygon([[p[0], p[1]] for p in point_list])

    def plot_cone(self, ax, clr=sp.Colors().sensor):
        """
        Plots the sensors detection cone

        :param ax: the given plot data series
        :type ax: plt.Axes
        :param clr: the colour of the cone
        :type clr: sp.Colors
        """
        if self.type: clr = sp.Colors().get_sensor_color(self.type)
        if self.cone in ax.patches:
            ax.patches.remove(self.cone)
        point_list = [(self.origin[0], self.origin[1])]
        for ray in self.detection_lines_list:
            point_list.append((ray.target[0], ray.target[1]))
        point_list.append((self.origin[0], self.origin[1]))

        path = Path(point_list)
        patch = patches.PathPatch(path, facecolor=clr, lw=0, zorder=8)
        ax.add_patch(patch)
        self.cone = patch
    
    def _delete_old(self, ax):
        for line in self.plot_lines:
            if line in ax.lines:
                ax.lines.remove(line)



class SensorRay:
    """
    Class representing a Sensor Ray.
    """
    def __init__(self, parent, target_point):
        """
        Constructor of a sensor ray.

        :param parent: the sensor, to which the ray belongs to
        :type parent: Sensor
        :param target_point: the ray's end point
        :type target_point: np.ndarray
        """
        self.length = parent.reach
        self.origin = parent.origin
        self.target = target_point
        self.target_type = ''
        self.temp_inter = []
        self.line = LineString([(parent.origin[0], parent.origin[1]), (target_point[0], target_point[1])])

    def rotate_ray(self, angle):
        """
        Rotates the ray around its origin by a certain angle given in degrees.

        :param angle: the angle to be rotated for in degrees
        :type angle: float
        """
        new_target = hf.rotate(self.origin, self.target, radians(-angle))
        self.target = new_target
        self.line = LineString([(self.origin[0], self.origin[1]), (new_target[0], new_target[1])])

    def move(self, delta):
        """
        Moves the sensor ray to a new position.

        :param delta: the values for which the ray is moved
        :type delta: np.ndarray
        """
        self.origin = self.origin + delta
        self.target = self.target + delta
        self.line = LineString([(self.origin[0], self.origin[1]), (self.target[0], self.target[1])])

    def choose_intersection(self):
        """
        Chooses the first intersection point of the ray making it it's new end point.
        """
        min_dist = inf
        for pt in self.temp_inter:
            new_dist = ((self.origin[0] - pt[0][0]) ** 2 + (self.origin[1] - pt[0][1]) ** 2) ** .5
            if new_dist < min_dist:
                self.target = np.array([pt[0][0], pt[0][1]])
                self.target_type = pt[1]
                min_dist = new_dist

        self.temp_inter = []

    def plot(self, ax, style='r-'):
        """
        Plots the ray to a given plot data series.

        :param ax: the plot data series
        :type ax: plt.Axes
        :param style: the plot style of the line
        :type style: str
        """
        x_values = [self.origin[0], self.target[0]]
        y_values = [self.origin[1], self.target[1]]
        sensor_line, = ax.plot(x_values, y_values, style, linewidth=.5)
        return sensor_line


class FieldOfView(Sensor):
    def __init__(self, parent=None, reach=0, angle=0, cover=100, direction=0, pos=np.array([0, 0])):
        super().__init__(parent=parent, reach=reach, angle=angle, cover=cover, direction=direction, pos=pos)
        self._point_list = []
        self.conflict_vehicles = []
        self.patch = None
        self.fancy_patches = []
        self.parent.field_of_vision = self

    def initialize(self):
        """
        Initializes the sensor, creating the cone.
        """
        self.is_vision_field = True
        turning_angle = self.direction + self.angle / 2

        delta_phi = self.angle / (self.coverage + 1)
        self._point_list.append((self.origin[0], self.origin[1]))
        for i in range(0, self.coverage + 2):
            x_i = self.origin[0] + self.reach * sin(radians(i * delta_phi - turning_angle + 90))
            y_i = self.origin[1] + self.reach * cos(radians(i * delta_phi - turning_angle + 90))
            self._point_list.append((x_i, y_i))

        self._point_list.append((self.origin[0], self.origin[1]))
        self.cone = Polygon([[p[0], p[1]] for p in self._point_list])

    def plot(self, ax, clr=sp.Colors().field_of_view):
        if self.patch in ax.patches:
            ax.patches.remove(self.patch)
            # plt.pause(.00000001)
        path = Path(self._point_list)
        patch = patches.PathPatch(path, facecolor=clr, lw=0)
        ax.add_patch(patch)
        self.patch = patch

    def update_sensor(self, next_junction=None):
        """
        Does an update for the sensor, calculating the new ray intersection points and the collected data.

        :return: returns the sensor collected border and car data
        :rtype: tuple[list[array], list[array]]
        :param borders: the borders of the current road
        :type borders: list[LineString]
        """
        self._point_list = []
        self.conflict_vehicles = []
        self.direction = self.start_direction + degrees(self.parent.angle)
        self.origin = self.get_origin()
        self.initialize()
        self.update_field_of_view(next_junction)


    def update_field_of_view(self, next_junction=None):
        self._point_list = []
        self.conflict_vehicles = []
        self.direction = self.start_direction + degrees(self.parent.angle)
        self.origin = self.get_origin()
        self.initialize()
        self.get_conflicting_agents(next_junction)
        agent_angle = self.parent.angle if self.parent.angle <= 0 else self.parent.angle - 360
        right_of_way_agents = []
        for conflict in self.conflict_vehicles:
            conflict_angle = traci.vehicle.getAngle(conflict)
            if abs((conflict_angle + agent_angle) % 90) - 90 <= sp.RIGHT_BEFORE_LEFT_EPS:
                right_of_way_agents.append(conflict)

        # shf.recolor_agent_list(right_of_way_agents, sp.Colors().critical_vehicle)
        return right_of_way_agents

    def get_conflicting_agents(self, next_junction=None):
        veh_list = traci.vehicle.getIDList()
        for veh in veh_list:
            if veh != self.parent.id:
                rect = shf.get_sumo_vehicle_as_poly(veh)
                for pt in rect.exterior.coords:
                    if Point(pt[0], pt[1]).within(self.cone):
                        # if next_junction and not junction_behind_agent(veh, next_junction):
                        # shf.recolor_agent(veh, sp.Colors().conflicting_vehicle)
                        self.conflict_vehicles.append(veh)
                        break
        # if len(self.conflict_vehicles) > 0:
        #     self.print_conflict_vehicles()


    def print_conflict_vehicles(self):
        print("------------------------ Conflicting Vehicles ----------------------")
        for veh in self.conflict_vehicles:
            print(veh + ": " + str(traci.vehicle.getPosition(veh)))
        print("--------------------------------------------------------------------")

def junction_behind_agent(conflict_veh, junction):
    pos_x = junction.pos_x
    pos_y = junction.pos_y
    agent_dir = shf.get_plt_angle(traci.vehicle.getAngle(conflict_veh))
    agent_pos = traci.vehicle.getPosition(conflict_veh)
    dir_to_junction = atan2((pos_y - agent_pos[1]), (pos_x - agent_pos[0]))
    test = abs(radians(agent_dir) - dir_to_junction)
    test2 = test % (2 * pi)
    if 3 / 2 * pi >  ((abs(radians(agent_dir) - dir_to_junction)) % (2 * pi)) > pi / 2:
        return True
    else:
        return False
    # if (abs(radians(agent_dir) - dir_to_junction) % (2 * pi)) < pi / 2:
    #     return False
    # else:
    #     return True


