import math
import random as rnd
import time

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.pyplot import Axes
from shapely.geometry import LineString, Point

import HelperFcts as hf
import PathPlanningTools.Base as base
import Sumo.SumoHelperFcts as shf
import Sumo.SumoParameters as sp

# ------------------------ DEFINES ----------------------
matplotlib.use("TkAgg")
color = hf.Colours()
# -------------------------------------------------------


class RRT:
    """
    RRT (Rapidly Random Exploring Trees) Super Class

    :param self.search_space_width: test
    """
    def __init__(self, position, rad, max_iterations=100, borders=None):
        """
        RRT Class Constructor

        :param borders: The borders of the current path
        :type borders: list[LineString]
        :param net: The given sumo-network, where the algorithm is operating - defining the search space
        :type net: CustomSumoNetVis.Net.Net
        :param position: starting position and facing angle of the agent
        :type position: base.Node
        :param rad: radius of the search space
        :type rad: float
        :param max_iterations: maximum algorithm iterations
        :type max_iterations: int
        """
        self.search_space_radius = rad
        self.n_max_iter = max_iterations
        self.conflicting_agents = []
        self.ball_radius = 25
        self.gamma = 20
        self.max_ball_r = 100
        self.agent_pos = position
        self.step_size = 5
        self.rrt_tree = None
        self.dimensions = 2
        self.borders = borders
        self.obstacle_func = self.obstacle_free
        self.sample_func = self.sample
        self.search_space = None
        self.configuration_space = None
        self.full_iterations = False
        self.plot_obstacles = False
        self.nodes_at_target = []

    def set_configuration_space(self, config):
        self.configuration_space = config

    def set_obstacles(self, obst_list):
        self.conflicting_agents = obst_list

    def set_obstacle_function(self, func):
        """
        Sets the obstacle function for the path finding algorithm

        :param func: the obstacle function to be used i.e. is_within_road
        :type func: object
        """
        self.obstacle_func = func

    def set_sample_func(self, func):
        """
        Sets the sample function for the sampled random Nodes

        :param func: the sampling function to be used i.e. informed_sample
        :type func: object
        """
        self.sample_func = func

    def initialize(self, start_node=None, end_node=None, tau=50):
        """
        Initializes the RRT path planning problem

        :param borders: List of LinesStrings defining the border of the current search space
        :type borders: list[LineString]
        :param start_node: starting node
        :type start_node: base.Node
        :param end_node: goal node, which should be reached
        :type end_node: base.Node
        :param tau: giving the step size ( edge length) between the nodes in the tree
        :type tau: float
        """
        # Get random start position, if not given
        if not start_node:
            start_node = self.sample()
            start_node.direction = 0
        start_node.nbr = 0
        start_node.type = 'start'
        if not end_node:
            end_node = self.sample()
            end_node.direction = 0
        # initialize tree
        self.step_size = tau


    def sample(self, start=None, goal=None, goal_rad=None):
        """
        Creates a random sample node within the given search space

        :return: random sample node without a direction. Direction is assigned in the main routine
        :rtype: base.Node
        """
        pos_x = rnd.uniform(-self.search_space_radius, self.search_space_radius)
        pos_y = rnd.uniform(-self.search_space_radius, self.search_space_radius)
        return base.Node(pos_x, pos_y)

    def informed_sample(self, start=None, goal=None, goal_rad=None):
        """
        Creates an informed sample node, which is inside the goal Circle

        :param goal: goal node
        :type goal: Node
        :param goal_rad: radius of the goal circle
        :type goal_rad: float
        :return: returns the informed sampled node without a direction. The direction is assigned in the main routine
        :rtype: base.Node
        """
        pos_x = rnd.uniform(goal.x - goal_rad, goal.x + goal_rad)
        pos_y = rnd.uniform(goal.y - goal_rad, goal.y + goal_rad)
        return base.Node(pos_x, pos_y)

    def bordered_sample(self, start=None, goal=None, goal_rad=None):
        """
        Creates a sample within the convex hull of the current path

        :return: the sampled node
        :rtype: base.Node
        """
        if math.pi / 2 > start.direction and start.direction > - math.pi / 2:
            pos_x = rnd.uniform(self.agent_pos.x, self.agent_pos.x + self.search_space_radius)
        else:
            pos_x = rnd.uniform(self.agent_pos.x - self.search_space_radius, self.agent_pos.x)
        pos_y = rnd.uniform(self.agent_pos.y - self.search_space_radius / 4, self.agent_pos.y + self.search_space_radius / 4)
        return base.Node(pos_x, pos_y)

    def search_space_sample(self, start=None, goal=None, goal_rad=None):
        pos_x = rnd.uniform(0, self.search_space[0])
        pos_y = rnd.uniform(0, self.search_space[1])
        return base.Node(pos_x, pos_y)

    def junction_sample(self, start=None, goal=None, goal_rad=None):
        angle = start.direction if start.direction > 0 else start.direction + 2 * math.pi

        if 0 < angle <= math.pi / 2:
            pos_x = rnd.uniform(self.agent_pos.x - self.search_space_radius, self.agent_pos.x + self.search_space_radius)
            pos_y = rnd.uniform(self.agent_pos.y, self.agent_pos.y + self.search_space_radius)
        elif math.pi / 2 < angle <= math.pi:
            pos_x = rnd.uniform(self.agent_pos.x - self.search_space_radius, self.agent_pos.x + self.search_space_radius)
            pos_y = rnd.uniform(self.agent_pos.y, self.agent_pos.y + self.search_space_radius)
        elif math.pi < angle <= 1.5 * math.pi:
            pos_x = rnd.uniform(self.agent_pos.x - self.search_space_radius, self.agent_pos.x + self.search_space_radius)
            pos_y = rnd.uniform(self.agent_pos.y - self.search_space_radius, self.agent_pos.y)
        else:
            pos_x = rnd.uniform(self.agent_pos.x - self.search_space_radius, self.agent_pos.x + self.search_space_radius)
            pos_y = rnd.uniform(self.agent_pos.y - self.search_space_radius, self.agent_pos.y)

        if Point(pos_x, pos_y).within(self.configuration_space):
            return base.Node(pos_x, pos_y)
        else:
            return self.junction_sample(start, goal, goal_rad)

    def nearest(self, node):
        """
        Gets the node nearest to the given node in the tree

        :param node: given node
        :type node: base.Node
        :return: nearest node in the tree
        :rtype: base.Node
        """
        dist = math.inf
        nearest_neighbour = base.Node()
        for neighbour in self.rrt_tree.nodes:
            if neighbour != node:
                new_dist = base.calc_node_dist(node, neighbour)
                if new_dist < dist:
                    dist = new_dist
                    nearest_neighbour = neighbour
        nearest_neighbour.type = 'nearest'
        return nearest_neighbour

    def steer(self, nearest_node, rand_node):
        """
        Gets a node along the path from nearest to random in a step-sized distance from nearest

        :param nearest_node: nearest node in the tree
        :type nearest_node: base.Node
        :param rand_node: samppled node to be reached
        :type rand_node: base.Node
        :return: returns the newly created node
        :rtype: base.Node
        """
        dist = base.calc_node_dist(nearest_node, rand_node)
        delta_y = (rand_node.y - nearest_node.y) * self.step_size / dist
        delta_x = (rand_node.x - nearest_node.x) * self.step_size / dist
        return base.Node(nearest_node.x + delta_x, nearest_node.y + delta_y, ttype='new')

    def obstacle_free(self, edge):
        """
        Checks whether a path crosses n obstacle or not

        :param edge: the edge to be evaluated
        :type edge: base.Edge
        :return: returns True if the edge does not cross an obstacle, otherwise False
        :rtype: bool
        """
        edge_as_linestring = edge.get_edge_as_linestring()
        for obstacle in self.conflicting_agents:
            obst_poly = shf.get_sumo_vehicle_as_poly(obstacle)
            if edge_as_linestring.intersects(obst_poly):
                return False
        return True

    def extended_obstacle_free(self, edge):
        edge_as_linestring = edge.get_edge_as_linestring()
        for obstacle in self.conflicting_agents:
            obst_poly = shf.get_sumo_vehicle_configuration_poly(obstacle)
            if edge_as_linestring.intersects(obst_poly):
                return False
        return True

    def obstacle_free2(self, edge):
        edge_as_linestring = edge.get_edge_as_linestring()
        if edge_as_linestring:
            for obstacle in self.conflicting_agents:
                if edge_as_linestring.intersects(obstacle):
                    return False
        else:
            return False
        return True

    def is_within_borders(self, edge):
        """
        Checks whether an edge is within the road borders or if it intersects with them

        :param edge: the edge segment to be evaluated
        :type edge: base.Edge
        :return: Returns True if the edge is fully within the road borders
        :rtype: bool
        """
        edge_as_linestring = edge.get_edge_as_linestring()
        for border in self.borders:
            if edge_as_linestring.intersects(border):
                return False
        return True

    def is_valid(self, edge):
        """
        Checks whether the new edge intersects with a lane border or an conflicting agent

        :param edge: the edge segment to be evaluated
        :type edge: base.Edge
        :return: Return True if the edge is fully within the border of the roads and does not intersect with a conflicting agent
        :rtype: bool
        """
        if self.is_within_borders(edge) and self.obstacle_free(edge):
            return True
        else:
            return False

    def junct_obstacle_fctn(self, edge):
        edge_as_linestring = edge.get_edge_as_linestring()
        if edge_as_linestring.within(self.configuration_space) and self.extended_obstacle_free(edge):
            return True
        else:
            return False

    def choose_parent(self, new_node, nearest_node):
        """
        Chooses a node form the tree as parent, which is closest to the new node within a given ball of radius
        self.ball_radius

        :param new_node: node for which a parent needs to be found
        :type new_node: base.Node
        :param nearest_node: current nearest node in the tree
        :type nearest_node: base.Node
        :return: returns all the nodes of tree within the specified ball
        :rtype: list[base.Node]
        """
        new_node.set_parent(nearest_node)
        ball_nodes = self.find_nodes_in_ball(new_node)
        min_cost = nearest_node.cost + base.calc_node_dist(new_node, nearest_node)
        for node in ball_nodes:
            new_edge = base.Edge(node, new_node)
            if self.obstacle_func(new_edge):
                new_cost = node.cost + new_edge.length
                if new_cost < min_cost:
                    new_node.set_parent(node)
                    min_cost = new_cost
        return ball_nodes

    def find_nodes_in_ball(self, centre_node):
        """
        Finds all the news within a ball around the centre node and the given radius self.ball_radius

        :param centre_node: the centre node
        :type centre_node: base.Node
        :return: returns all the nodes in the ball
        :rtype: list[base.Node]
        """
        ball_nodes = []
        for node in self.rrt_tree.nodes:
            if node != centre_node:
                if base.calc_node_dist(centre_node, node) < self.ball_radius:
                    ball_nodes.append(node)
        return ball_nodes

    def rewire(self, new_node, ball_nodes):
        """
        Rewires the tree, checking whether some nodes in the tree can be reached for a lower cost, making the new node
        their parents

        :param new_node: newly added node and parent candidate
        :type new_node: base.Node
        :param ball_nodes: all the nodes to be evaluated for rewiring
        :type ball_nodes: list[base.Node]
        :return: returns True if the tree has been rewired, else False
        :rtype: bool
        """
        rewired = False
        for node in ball_nodes:
            if node.cost > new_node.cost + hf.calc_point_dist(node, new_node):
                node.parent = new_node
                rewired = True
        new_node.type = 'fixed'
        return rewired

    def get_shrinking_ball_radius(self):
        """
        Calculates the current ball radius.

        :return ball_r: returns the current ball radius
        :rtype: float
        """
        return self.ball_radius
        # nbr_of_nodes = len(self.rrt_tree.nodes)
        # return min(self.gamma * (math.log(nbr_of_nodes)) / nbr_of_nodes, self.max_ball_r)

    def get_gamma(self):
        """
        Calculates the current gamma value.
        """
        self.gamma = (2 * (1 + 1 / self.dimensions)) ** self.dimensions * ()

    def get_shortest_path(self):
        if len(self.nodes_at_target) == 0:
            return None, None
        cost = math.inf
        cheapest_node = None
        for node in self.nodes_at_target:
            new_cost = node.cost
            if new_cost < cost:
                cost = new_cost
                cheapest_node = node

        return cheapest_node, cost

    def rrt(self, start_node=None, end_node=None, goal_rad=10, tau=5, ball_radius=25, ax=None):
        """
        Main RRT path planning algorithm

        :param start_node: starting node
        :type start_node: base.Node
        :param end_node: goal node to be reached by the algorithm
        :type end_node: base.Node
        :param goal_rad: radius of the goal circle
        :type goal_rad: float
        :param tau: step length between the nodes in the tree
        :type tau: float
        :param ax: plot data series for real time plotting, if None, nothing is plotted
        :type ax: Axes
        """
        # Get random start position, if not given
        self.initialize(start_node, end_node, tau)
        self.rrt_tree = base.Tree(start_node, end_node, goal_rad, ax)
        self.ball_radius = ball_radius

        # initialize variables
        rand, rand_line = None, None

        for i in range(0, self.n_max_iter):
            rand_node = self.sample()
            if ax:
                rand = rand_node.plot(color.tree_rand_nd, ax)
                plt.pause(0.000001)
            nearest = self.nearest(rand_node)
            if ax:
                near = nearest.plot(color.tree_nearest_nd, ax)
                rand_line, = ax.plot([nearest.x, rand_node.x], [nearest.y, rand_node.y], 'k--', linewidth=.7)
                plt.pause(0.000001)
            new_node = self.steer(nearest, rand_node)
            new_edge = base.Edge(nearest, new_node)
            # if new_edge not in obstacle
            if self.obstacle_free(new_edge):
                new_node.nbr = i + 1
                new_node.set_parent(nearest)
                new = self.rrt_tree.insert_node(new_node, ax)
                if ax:
                    plt.pause(0.000001)
                    ax.patches.remove(rand)
                    ax.lines.remove(rand_line)
                    self.rrt_tree.rewired = True
                    self.rrt_tree.plot(ax, new_node)
                    new.set_facecolor(color.tree_nd)
                    plt.pause(0.000001)

                if end_node and base.node_in_circle(end_node, goal_rad, new_node):
                    self.rrt_tree.plot_target_path(new_node, ax)
                    break

    def rrt_star(self, start_node=None, end_node=None, goal_rad=10, tau=100, informed=False, ball_radius=50, ax=None):
        """
        Main RRT* path planning algorithm

        :param start_node: starting node
        :type start_node: base.Node
        :param end_node: goal node to be reached by the algorithm
        :type end_node: base.Node
        :param goal_rad: radius of the goal circle
        :type goal_rad: float
        :param tau: step length between the nodes in the tree
        :type tau: float
        :param informed: if True, all the samples will be close to the goal
        :type informed: bool
        :param ax: plot data series for real-time plot. If ax is none, then nothing is plotted
        :type ax: Axes
        :return: Returns the Node, which was within the goal radius
        :rtype: base.Node
        """
        if informed:
            self.sample_func = self.informed_sample

        rand, rand_line, ball = None, None, None
        # Get random start position, if not given
        self.initialize(start_node, end_node, tau)
        self.ball_radius = ball_radius
        self.rrt_tree = base.Tree(start_node, end_node, goal_rad, ax)

        for i in range(0, self.n_max_iter):
            rand_node = self.sample_func(start_node, end_node, goal_rad)
            if ax:
                rand = rand_node.plot(color.tree_rand_nd, ax, annotate=False)
                plt.pause(0.000001)
            nearest = self.nearest(rand_node)
            if ax:
                near = nearest.plot(color.tree_nearest_nd, ax)
                rand_line, = ax.plot([nearest.x, rand_node.x], [nearest.y, rand_node.y], 'k--', linewidth=.7)
                plt.pause(0.000001)
            new_node = self.steer(nearest, rand_node)
            new_edge = base.Edge(nearest, new_node)
            # if new_edge not in obstacle
            if self.obstacle_func(new_edge):
                new_node.nbr = i + 1
                new = self.rrt_tree.insert_node(new_node, ax)
                if ax:
                    ball = matplotlib.patches.Circle((new_node.x, new_node.y), self.ball_radius, linestyle='--',
                                                     fill=False, linewidth=.5)
                    ax.add_patch(ball)
                    plt.pause(0.000001)
                ball_nodes = self.choose_parent(new_node, nearest)
                self.rrt_tree.rewired = self.rewire(new_node, ball_nodes)
                if ax:
                    # ax.patches.remove(rand)
                    # ax.lines.remove(rand_line)
                    # ax.patches.remove(ball)
                    self.rrt_tree.plot(ax, new_node)
                    new.set_facecolor(color.tree_nd)
                    plt.pause(0.000001)

                if end_node and base.node_in_circle(end_node, goal_rad, new_node):
                    if ax:
                        self.rrt_tree.plot_target_path(new_node, ax)
                    if new_node.nbr == 0:
                        break
                    if not self.full_iterations:
                        return new_node
                    else:
                        self.nodes_at_target.append(new_node)

            if ax:
                if near in ax.patches:
                    ax.patches.remove(near)
                if rand in ax.patches:
                    ax.patches.remove(rand)
                if rand_line in ax.lines:
                    ax.lines.remove(rand_line)
                if ball in ax.patches:
                    ax.patches.remove(ball)


class DubinsRRT(RRT):
    """
    Dubins RRT (Rapidly Random Exploring Trees using Dubins Paths as Edges) Child Class
    """
    def __init__(self, position, rad, max_iters, borders=None):
        """
        Dubins RRT Constructor

        :param net: The given sumo-network, where the algorithm is operating - defining the search space
        :type net: CustomSumoNetVis.Net.Net
        :param position: starting position and facing angle of the agent
        :type position: base.Node
        :param rad: radius of the search space
        :type rad: float
        :param max_iters: maximum algorithm iterations
        :type max_iters: int
        """
        super().__init__(position, rad, max_iterations=max_iters, borders=borders)
        self._agent_curvature = sp.MIN_CURVATURE

    def set_curvature(self, c):
        self._agent_curvature = c

    def rrt(self, start_node=None, end_node=None, goal_rad=10, tau=100, dt=.1, informed=False, ax=None):
        """
        Main RRT path planning algorithm

        :param start_node: starting node
        :type start_node: base.Node
        :param end_node: goal node to be reached by the algorithm
        :type end_node: base.Node
        :param goal_rad: radius of the goal circle
        :type goal_rad: float
        :param tau: step length between the nodes in the tree
        :type tau: float
        :param dt: time increment given in seconds
        :type dt: float
        :param informed: if True, all the samples will be close to the goal
        :type informed: bool
        :param ax: plot data series for real-time plot. If ax is none, then nothing is plotted
        :type ax: Axes
        """
        # initialize tree and plots
        super().initialize(start_node, end_node, tau)
        self.rrt_tree = base.DubinsTree(start_node, end_node, goal_rad, tau, ax)
        rand, rand_line = None, None

        for i in range(0, self.n_max_iter):
            if informed and end_node:
                rand_node = super().informed_sample(end_node, goal_rad)
            else:
                rand_node = super().sample()
            if ax:
                rand = rand_node.plot(color.tree_rand_nd, ax)
                plt.pause(0.000001)
            nearest = super().nearest(rand_node)

            # get dusbins path from nearest to rand of length tau
            rand_node.direction = math.atan2((rand_node.y - nearest.y), (rand_node.x - nearest.x))
            dubins_edge = base.DubinsEdge(nearest, rand_node, dt, tau)

            if ax:
                near = nearest.plot(color.tree_nearest_nd, ax)
                complete_edge = base.DubinsEdge(nearest, rand_node, dt, tau)
                rand_line, = ax.plot(complete_edge.segment[0], complete_edge.segment[1], 'k-', linewidth=.7)
                plt.pause(0.000001)

            new_node = dubins_edge.dest
            new_node.type = 'new'

            # if new_edge not in obstacle
            if self.obstacle_free(dubins_edge):
                new_node.nbr = i + 1
                new_node.set_parent(nearest, dubins_edge)
                new = self.rrt_tree.insert_node(new_node, ax)
                if ax:
                    plt.pause(0.000001)
                    ax.patches.remove(rand)
                    ax.lines.remove(rand_line)
                    self.rrt_tree.rewired = True
                    self.rrt_tree.draw_tree(ax, new_node)
                    new.set_facecolor(color.tree_nd)
                    plt.pause(0.000001)



            if end_node and base.node_in_circle(end_node, goal_rad, new_node):
                if ax:
                    self.rrt_tree.plot_target_path(new_node, ax)
                break


    def rrt_star(self, start_node=None, end_node=None, goal_rad=10, tau=100, dt=sp.TIME_STEP, informed=False,
                 running_time=math.inf, ball_radius=10, ax=None):
        """
        Main RRT* path planning algorithm

        :param start_node: starting node
        :type start_node: base.Node
        :param end_node: goal node to be reached by the algorithm
        :type end_node: base.Node
        :param goal_rad: radius of the goal circle
        :type goal_rad: float
        :param tau: step length between the nodes in the tree
        :type tau: float
        :param dt: time increment given in seconds
        :type dt: float
        :param informed: if True, all the samples will be close to the goal
        :type informed: bool
        :param running_time: time interval for which the algorithm is run given in seconds
        :type running_time: float
        :param ax: plot data series for real-time plot. If ax is none, then nothing is plotted
        :type ax: Axes
        """
        # initialize tree and plots
        if ball_radius:
            self.ball_radius = ball_radius
        super().initialize(start_node, end_node, tau)
        self.rrt_tree = base.DubinsTree(start_node, end_node, goal_rad, tau, ax)
        rand, rand_line, ball = None, None, None

        start_time = time.clock()
        for i in range(0, self.n_max_iter):
            rand_node = self.sample_func(start_node, end_node, goal_rad)
            # if informed and end_node:
            #     rand_node = super().informed_sample(end_node, goal_rad)
            # else:
            #     rand_node = super().sample()
            if ax:
                rand = rand_node.plot(color.tree_rand_nd, ax)
                plt.pause(0.000001)
            nearest = super().nearest(rand_node)

            # get dubins path from nearest to rand of length tau
            rand_node.direction = math.atan2((rand_node.y - nearest.y), (rand_node.x - nearest.x))
            dubins_edge = base.DubinsEdge(nearest, rand_node, dt, tau, agent_curvature=self._agent_curvature)
            if dubins_edge.is_invalid:
                continue

            if ax:
                near = nearest.plot(color.tree_nearest_nd, ax)
                complete_edge = base.DubinsEdge(nearest, rand_node, dt, agent_curvature=self._agent_curvature, time_int=0)
                rand_line, = ax.plot(complete_edge.segment[0], complete_edge.segment[1], 'k-', linewidth=.7)
                plt.pause(0.000001)

            new_node = dubins_edge.dest
            new_node.type = 'new'

            # if new_edge not in obstacle
            if self.obstacle_func(dubins_edge):
                new_node.nbr = i + 1
                new = self.rrt_tree.insert_node(new_node, ax)
                if ax:
                    ball = matplotlib.patches.Circle((new_node.x, new_node.y), self.ball_radius, linestyle='--',
                                                     fill=False, linewidth=.5)
                    ax.add_patch(ball)
                    plt.pause(0.000001)
                ball_nodes = self.choose_parent(new_node, nearest, dubins_edge)
                self.rrt_tree.rewired = self.rewire(new_node, ball_nodes)
                if ax:
                    # ax.patches.remove(rand)
                    # ax.lines.remove(rand_line)
                    # ax.patches.remove(ball)
                    self.rrt_tree.draw_tree(ax, new_node)
                    new.set_facecolor(color.tree_nd)
                    ax.patches.remove(near)
                    plt.pause(0.000001)



                if end_node and base.node_in_circle(end_node, goal_rad, new_node):
                    if ax:
                        if near in ax.patches:
                            ax.patches.remove(near)
                        if rand in ax.patches:
                            ax.patches.remove(rand)
                        if rand_line in ax.lines:
                            ax.lines.remove(rand_line)
                        if ball in ax.patches:
                            ax.patches.remove(ball)
                        self.rrt_tree.plot_target_path(new_node, ax)
                        plt.pause(0.000001)
                    break

            if ax:
                if near in ax.patches:
                    ax.patches.remove(near)
                if rand in ax.patches:
                    ax.patches.remove(rand)
                if rand_line in ax.lines:
                    ax.lines.remove(rand_line)
                if ball in ax.patches:
                    ax.patches.remove(ball)

            if time.clock() - start_time >= running_time:
                return self.rrt_tree.find_node_closest_to_goal()


    def segmented_rrt_star(self, start_node=None, end_node=None, goal_rad=10, tau=100, dt=.1, informed=False,
                           running_time=2.0, ax=None):
        """
        Segmented RRT* path planning algorithm

        :param start_node: starting node
        :type start_node: base.Node
        :param end_node: goal node to be reached by the algorithm
        :type end_node: base.Node
        :param goal_rad: radius of the goal circle
        :type goal_rad: float
        :param tau: step length between the nodes in the tree
        :type tau: float
        :param dt: time increment given in seconds
        :type dt: float
        :param informed: if True, all the samples will be close to the goal
        :type informed: bool
        :param ax: plot data series for real-time plot. If ax is none, then nothing is plotted
        :param running_time: time for which the rrt is running until the next commitment
        :type running_time: float
        :type ax: Axes
        """
        # initialize agent
        agent = base.Agent(start_node)
        agent.plot_agent(ax)
        # initial path finding run
        initializing_time = 3
        queue = []
        current_closest = self.rrt_star(start_node, end_node, goal_rad, tau, dt, informed, initializing_time, None)
        queue.append(current_closest)
        self.rrt_tree.draw_tree(ax)
        agent.move_along_path(end_node=queue[0], ax=ax)
        # Start driving and further plan the tree
        # pool = multiprocessing.Pool()






    def obstacle_free(self, dubins_edge):
        """
        Checks whether a path crosses an obstacle - simple approach

        :param dubins_edge: the dubins edge to be evaluated
        :type dubins_edge: base.DubinsEdge
        :return: returns True if the path is obstacle free, else False
        :rtype: bool
        """
        for obstacle in self.conflicting_agents:
            veh_poly = shf.get_sumo_vehicle_as_poly(obstacle)
            if dubins_edge.linestring_segment.intersects(veh_poly):
                    return False
        return True


    def choose_parent(self, new_node, nearest_node, dubins_path=None):
        """
        Chooses the parent for the new node with the shortest dubins path to the parent

        :param new_node: new node for which a parent needs to be found
        :type new_node: base.Node
        :param nearest_node: currently nearest node in the tree
        :type nearest_node: base.Node
        :param dubins_path: currently selected dubins path
        :type dubins_path: base.DubinsEdge
        :param c: maximum curvature of the agent
        :type c: float
        :return: nodes in the ball around the new node using self.ball_radius as the ball radius
        :rtype: list[base.Node]
        """
        new_node.set_parent(nearest_node, dubins_path)
        ball_nodes = self.find_nodes_in_ball(new_node)
        min_cost = new_node.cost
        for node in ball_nodes:
            # TODO:
            # I could change the direction of new_node here, so that it comes from node, makes the paths shorter in
            # general
            new_direction = math.atan2((new_node.y - node.y), (new_node.x - node.x))

            new_edge = base.DubinsEdge(node, new_node, agent_curvature=self._agent_curvature)
            if not new_edge.linestring_segment == [] and self.obstacle_func(new_edge):
                new_cost = node.cost + new_edge.length
                if new_cost < min_cost:
                    new_node.set_parent(node, new_edge)
                    min_cost = new_cost
        return ball_nodes

    def rewire(self, new_node, ball_nodes):
        """
        Rewires the tree, checking whether some nodes in the tree can be reached for a lower cost, making the new node
        their parents

        :param new_node: newly added node and parent candidate
        :type new_node: base.Node
        :param ball_nodes: all the nodes to be evaluated for rewiring
        :type ball_nodes: list[base.Node]
        :param c: maximum curvature of the agent
        :type c: float
        :return: returns True if the tree has been rewired, else False
        :rtype: bool
        """
        rewired = False
        for node in ball_nodes:
            new_edge = base.DubinsEdge(new_node, node, agent_curvature=self._agent_curvature)
            if node.cost > new_node.cost + new_edge.length:
                node.set_parent(new_node, new_edge)
                rewired = True
        new_node.type = 'fixed'
        return rewired
