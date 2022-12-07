# implementation of AStar Algorithm
import copy

import matplotlib.patches
import matplotlib.pyplot as plt
import shapely.ops
from matplotlib.path import Path
from shapely.geometry import LineString, MultiLineString

import HelperFcts as hf
import Paths as params
import Sumo.SumoHelperFcts as shf
import Sumo.SumoParameters as sp
from PathPlanningTools import Base, MyRRT

# ------------------------ DEFINES ----------------------
color = hf.Colours()
network_settings = params.NETWORK("ValidationNetwork.net.xml")


# -------------------------------------------------------

# TODO: Uses wrong SUMO-Net needs to be updated!!!!

class Navigator():
    """
    Main Class for the Navigator of the Autonomous vehicle
    """

    def __init__(self, net):
        """
        Navigator Class Constructor using an A* planner to navigate

        :param net: The current network storing all the nodes and edges
        :type net: SumoNetVis.Net
        """
        self.nodes = []
        self.edges = []
        self.sumo_net = net
        self.connection_borders = dict()
        self.initialize()

    def initialize(self, cars_only=True):
        """
        Initializes the current network, with lanes represented as nodes an connections as edges.

        :param cars_only: only adds nodes and edges to the network, which are accessible by cars
        :type cars_only: bool
        """
        add_lane = True
        i = 0
        self.sumo_net.add_missing_connections()

        for edge in self.sumo_net.edges:
            for lane in edge.lanes:
                lane.get_lane_as_node()
                if cars_only:
                    add_lane = str.__eq__(lane.allow, '') and not str.__eq__(lane.disallow, 'all')
                if add_lane and lane.parentEdge.function != 'internal':
                    self.nodes.append(Base.Node(lane.centre[0], lane.centre[1], 0))
                    self.nodes[-1].nbr = i
                    self.nodes[-1].set_length_and_lane_id(lane.length, lane.id)
                    i += 1

        add_connection = True
        for connection in self.sumo_net.connections:
            if cars_only:
                add_connection = str.__eq__(connection.from_lane.allow, '') \
                                 and not str.__eq__(connection.from_lane.disallow, 'all') \
                                 and str.__eq__(connection.to_lane.allow, '') \
                                 and not str.__eq__(connection.to_lane.disallow, 'all')
            if add_connection:
                retVal = connection.get_connection_as_edge()
                if retVal and connection.to_edge.function != 'internal' and connection.from_edge.function != 'internal':
                    start_node = Base.Node(retVal[0], retVal[1])
                    start_node.set_length_and_lane_id(connection.from_lane.length, connection.from_lane.id)
                    end_node = Base.Node(retVal[2], retVal[3])
                    end_node.set_length_and_lane_id(connection.to_lane.length, connection.to_lane.id)
                    self.edges.append(Base.Edge(start_node, end_node))

    def plot(self, ax):
        """
        Plots the current connection network, represented in nodes and edges view

        :param ax: plot data series
        :type ax: plt.Axes
        """
        for edge in self.edges:
            edge.plot(clr=hf.Colours().mpl_line_green, ax=ax)

        for node in self.nodes:
            node.plot(clr=color.mpl_line_red, r=3, ax=ax)

    def plot_target_path(self, successful_node, ax, route=None, connections=None):
        if route:
            for lane in route:
                real_lane = network_settings.net.get_lane_by_id(lane)
                real_lane.plot_alignment(ax, 'r-')
            for connect in connections:
                connect.plot_alignment(ax)
        else:
            while successful_node.parent:
                ax.plot([successful_node.x, successful_node.parent.x], [successful_node.y, successful_node.parent.y], 'g-', linewidth=3)
                successful_node = successful_node.parent

    def AStar(self, start_lane_id, goal_lane_id):
        """
        Main A* Navigation algorithm to be used within a SUMO network

        :param start_lane_id: id of the starting lane
        :type start_lane_id: str
        :param goal_lane_id: id of the desired lane
        :type goal_lane_id: str
        :return: returns the succeeding node/lane
        :rtype: Base.Node
        """
        start_lane = self.sumo_net.get_lane_by_id(start_lane_id)
        goal_lane = self.sumo_net.get_lane_by_id(goal_lane_id)

        if start_lane and goal_lane:
            start_lane.get_lane_as_node()
            goal_lane.get_lane_as_node()
            start_node = Base.Node(start_lane.centre[0], start_lane.centre[1])
            start_node.set_length_and_lane_id(start_lane.length, start_lane_id)
            goal_node = Base.Node(goal_lane.centre[0], goal_lane.centre[1])
            goal_node.set_length_and_lane_id(goal_lane.length, goal_lane_id)
        else:
            print("Start or Goal Lane not found!")
            return None

        net_extents = self.sumo_net._get_extents()
        closed_set = []
        open_set = [start_node]
        #         t_break = 1 + 1 / (abs(net_extents[0] - net_extents[2]) + abs(net_extents[1] - net_extents[3]))
        t_break = 1  + 1 /len(self.nodes)

        # Estimated total cost from start to goal:
        start_node.f = Base.calc_node_dist(start_node, goal_node)

        while len(open_set) > 0:
            current_node = open_set[0]
            current_index = 0
            for index, item in enumerate(open_set):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            if current_node.lane_id == goal_node.lane_id:
                return current_node

            open_set.pop(current_index)  # remove current node from open_set
            closed_set.append(current_node)  # add current node to closed_set

            for neighbour in self.find_neighbours(current_node):
                # if neighbour is in closed_set
                for closed_neighbour in closed_set:
                    if neighbour == closed_neighbour:
                        continue

                # calculate tentative_g_score
                tentative_g_score = current_node.g + current_node.length

                if not self.neighbour_in_open_set(neighbour, open_set) or tentative_g_score < neighbour.g:
                    neighbour.parent = current_node
                    neighbour.g = tentative_g_score
                    neighbour.f = neighbour.g + t_break * Base.calc_node_dist(neighbour, goal_node)
                    if not self.neighbour_in_open_set(neighbour, open_set):
                        open_set.append(neighbour)
        raise hf.OutOfReachError("No path from Start to Goal could be found!")

    def get_route(self, start_lane_id, goal_lane_id):
        successful_node = self.AStar(start_lane_id, goal_lane_id)
        if successful_node:
            return (self.get_itinerary(successful_node))
        else:
            return None

    def find_neighbours(self, current_node):
        """
        Finds all the neighbours of the current node

        :param current_node: the current node/lane
        :type current_node: Base.Node
        :return: All the neighbours of the given node
        :rtype: list[Base.Node]
        """
        neighbours = []
        for edge in self.edges:
            if current_node.lane_id == edge.start_node.lane_id:
                neighbours.append(edge.end_node)
        return neighbours

    def neighbour_in_open_set(self, neighbour, open_set):
        """
        Checks whether a neighbour is within the current open set

        :param neighbour: current neighbour
        :type neighbour: Base.Node
        :param open_set: current open set
        :type open_set: list[Base.Node]
        :return: True if in open set else False
        :rtype: bool
        """
        for item in open_set:
            if neighbour == item:
                return True
        return False

    def get_itinerary(self, successful_node, ax=None):
        """
        Plots the path from the start to the target node

        :return: returns a list of lanes from start to goal
        :rtype: list[CustomSumoNetVis.Net.Lane]
        :param successful_node: end node of the plotted path, usually the node within the goal circle
        :type successful_node: Node
        :param ax: plot data series
        :type ax: Axes
        """
        target_path = []
        connections = []
        target_path_ids = []
        while successful_node:
            target_path_ids.append(successful_node.lane_id)
            successful_node = successful_node.parent
        target_path_ids.reverse()

        for id in target_path_ids:
            for edge in self.sumo_net.edges:
                for lane in edge.lanes:
                    if lane.id == id:
                        target_path.append(lane)

        for i in range(0, len(target_path) - 1):
            con = network_settings.net.get_connection(target_path[i], target_path[i + 1])
            connections.append(con)

        borders, connection_borders = self.sumo_net.get_borders(target_path, ax)
        return borders, target_path_ids, connections, connection_borders

    def highlight_path(self, borders, ax, style='w-.', borders_only=False):
        """
        Plots the given borders of a given path

        :param borders: List of LineStrings to be plotted
        :type borders: list[LineString]
        :param ax: plot data series
        :type ax: plt.Axes
        :param style: stype of the plotted border lines
        :type style: str
        """
        if borders_only:
            for line in borders:
                x, y = zip(*line.coords)
                ax.plot(x, y, style, linewidth=2, zorder=2)
        else:
            line_list = []
            for line in borders:
                line_list.append(line)
            multi_line = MultiLineString(line_list)
            merged_line = shapely.ops.linemerge(multi_line)
            coords = []
            for pt in merged_line[0].coords:
                coords.append(pt)
            for pt in merged_line[1].coords:
                coords.append(pt)
            path = Path(coords)
            plt_patch = matplotlib.patches.PathPatch(path, facecolor=sp.Colors().route, lw=0, zorder=2)
            ax.add_patch(plt_patch)

    def get_path_evaluation_points(self, start_lane_id, goal_lane_id, start_pos, end_pos, ax=None):

        print("Looking for route...")
        successful_node = self.AStar(start_lane_id, goal_lane_id)
        print("Found route!")
        if successful_node:
            borders, path = self.get_itinerary(successful_node)
        else:
            return

        if ax:
            self.highlight_path(borders, ax)
            plt.pause(.1)

        rrt = MyRRT.RRT(start_pos, 1000, max_iterations=100000, borders=borders)
        rrt.set_obstacle_function(rrt.is_within_borders)
        rrt.set_sample_func(rrt.bordered_sample)
        print("Looking for path...")
        succeeding_node = rrt.rrt_star(start_pos, end_node=end_pos, tau=2, ball_radius=10)
        succeeding_node.direction = start_pos.direction
        print("Found path!")

        rrt.rrt_tree.renumber_target_path(succeeding_node)
        return create_dubins_path(borders, start_pos, succeeding_node, ax=ax)

    def plot_route(self, route, ax):
        for dubins_edge in route:
            dubins_edge.plot(ax)





def get_extents(path, ax=None):
    lane_geoms = []
    for lane in path:
        lane_geoms.append(lane)
    polygons = MultiLineString(lane_geoms)

    if ax:
        x_values = (polygons.bounds[0], polygons.bounds[0], polygons.bounds[2], polygons.bounds[2], polygons.bounds[0])
        y_values = (polygons.bounds[1], polygons.bounds[3], polygons.bounds[3], polygons.bounds[1], polygons.bounds[1])
        ax.plot(x_values, y_values, 'k--')

    return polygons.bounds


def create_dubins_path(borders, start_node, successful_node, ax=None, conflict_agents=None, use_polygon=False):
    nbr_of_nodes = successful_node.nbr
    tree = Base.DubinsTree(start=None, goal=None, rad=None, time_int=100, ax=ax)

    current_node = copy.copy(successful_node)
    while current_node.parent:
        current_node.redirect(current_node.parent)
        current_node.parent.set_child(current_node)
        tree.insert_node(current_node, ax=ax)
        current_node = current_node.parent

    tree.insert_node(start_node, ax=ax)
    tree.start_node = tree.get_node_by_nbr(0)
    tree.goal_node = tree.get_node_by_nbr(nbr_of_nodes)

    # keep separating the path until it does not intersect with the borders anymore
    tree.beacons, path = optimize_path(start_node, successful_node, borders, ax=ax, conflict_agents=conflict_agents,
                                       use_polygon=use_polygon)
    if len(tree.beacons) > 0:
        last_edge = Base.DubinsEdge(tree.start_node, tree.beacons[-1], agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
        path.insert(0, last_edge)
    if ax:
        for element in path:
            element.plot(ax)
        tree.plot_beacons(ax, radius=5)
        plt.pause(.1)

    x_complete = []
    y_complete = []
    for i  in range(0, len(path)):
        x_complete.extend(path[i].segment[0])
        y_complete.extend(path[i].segment[1])

    complete = []
    for i in range (0, len(x_complete)):
        complete.append((x_complete[i], y_complete[i]))

    return LineString(complete)
    # ax.plot(dubins_path.segment[0], dubins_path.segment[1], 'r-')


def optimize_path(start_node, successful_node, borders, ax=None, conflict_agents=None, use_polygon=False):
    navigator = Navigator(network_settings.net)
    if ax:
        navigator.highlight_path(borders, ax)
    beacons = []
    path = []
    visible_node = successful_node.parent
    vn_plot, succ_plot = None, None
    last_free_node = None

    if use_polygon:
        return optimize_polygon_path(start_node, successful_node, borders, ax, conflict_agents)
    else:
        complete_edge = Base.DubinsEdge(start_node, successful_node, agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
        complete_edge_as_linestring = complete_edge.get_edge_as_linestring(simplified=True)
        if fully_inside(complete_edge_as_linestring, borders):
            path.append(complete_edge)
            path.reverse()
            return beacons, path
        else:
            if visible_node:
                while visible_node.parent:
                    if ax and vn_plot and succ_plot:
                        ax.patches.remove(vn_plot)
                        ax.patches.remove(succ_plot)
                        plt.pause(.1)

                    visible_node.redirect(successful_node)
                    if ax:
                        vn_plot = visible_node.plot(clr=color.mpl_line_red, ax=ax, r=.5)
                        succ_plot = successful_node.plot(clr=color.mpl_line_blue, ax=ax, r=.5)
                        plt.pause(.1)

                    new_edge = Base.DubinsEdge(visible_node, successful_node, agent_curvature=sp.MIN_CURVATURE,
                                               dt=sp.TIME_STEP)
                    # new_edge_as_linestring = new_edge.get_edge_as_linestring()
                    new_edge_as_linestring = new_edge.get_edge_as_linestring(simplified=True)
                    if fully_inside(new_edge_as_linestring, borders, conflict_agents=conflict_agents):
                        last_free_node = visible_node
                        successful_node.set_parent(visible_node, None)

                    else:
                        latest_valid_edge = Base.DubinsEdge(visible_node.child, successful_node,
                                                            agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
                        path.append(latest_valid_edge)
                        if ax:
                            latest_valid_edge.plot(ax)
                            plt.pause(.1)

                        if last_free_node:
                            beacons.append(last_free_node)
                            successful_node = last_free_node
                            visible_node = successful_node
                        else:
                            # return beacons, path
                            break
                    visible_node = visible_node.parent
                    print("optimizing...")
            path.reverse()
            last_edge = Base.DubinsEdge(start_node, beacons[-1], agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
            path.insert(0, last_edge)
            return beacons, path

def optimize_polygon_path(start_node, successful_node, polygon, ax=None, conflict_agents=None):
    beacons = []
    path = []
    visible_node = successful_node.parent
    vn_plot, succ_plot = None, None
    last_free_node = None

    complete_edge = Base.DubinsEdge(start_node, successful_node, agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
    complete_edge_as_linestring = complete_edge.get_edge_as_linestring(simplified=False)
    if is_valid(complete_edge_as_linestring, polygon, conflict_agents):
        path.append(complete_edge)
        path.reverse()
        return beacons, path
    else:
        if visible_node:
            while visible_node.parent:
                if ax and vn_plot and succ_plot:
                    ax.patches.remove(vn_plot)
                    ax.patches.remove(succ_plot)
                    plt.pause(.1)

                visible_node.redirect(successful_node)
                if ax:
                    vn_plot = visible_node.plot(clr=color.mpl_line_red, ax=ax, r=.5)
                    succ_plot = successful_node.plot(clr=color.mpl_line_blue, ax=ax, r=.5)
                    plt.pause(.1)

                new_edge = Base.DubinsEdge(visible_node, successful_node, agent_curvature=sp.MIN_CURVATURE,
                                           dt=sp.TIME_STEP)
                # new_edge_as_linestring = new_edge.get_edge_as_linestring()
                new_edge_as_linestring = new_edge.get_edge_as_linestring(simplified=True)
                if is_valid(new_edge_as_linestring, polygon, conflict_agents):
                    last_free_node = visible_node
                    successful_node.set_parent(visible_node, None)

                else:
                    latest_valid_edge = Base.DubinsEdge(visible_node.child, successful_node,
                                                        agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
                    path.append(latest_valid_edge)
                    if ax:
                        latest_valid_edge.plot(ax)
                        plt.pause(.1)

                    if last_free_node:
                        beacons.append(last_free_node)
                        successful_node = last_free_node
                        visible_node = successful_node
                    else:
                        # return beacons, path
                        break
                visible_node = visible_node.parent
                print("optimizing...")
        path.reverse()
        last_edge = Base.DubinsEdge(start_node, beacons[-1], agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
        path.insert(0, last_edge)
        return beacons, path


def fully_inside(edge, borders, conflict_agents=None):
    if conflict_agents:
        for veh in conflict_agents:
            veh_rect = shf.get_sumo_vehicle_as_poly(veh)
            if edge.intersects(veh_rect):
                return False

    for border in borders:
        if edge.intersection(border):
            return False

    return True

def is_valid(edge, polygon, conflict_agents=None):
    if conflict_agents:
        for veh in conflict_agents:
            veh_rect = shf.get_sumo_vehicle_as_poly(veh)
            if edge.intersects(veh_rect):
                return False

    if not edge.within(polygon):
        return False

    return True


def get_coords_as_linestring(px, py):
    points = []
    for i in range(0, len(px)):
        points.append([px[i], py[i]])
    return LineString(points)
