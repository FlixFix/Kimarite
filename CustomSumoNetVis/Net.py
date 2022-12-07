"""
Main classes and functions for dealing with a Sumo network.

Author: Patrick Malcolm
"""

import random
import xml.etree.ElementTree as ET
from math import atan2, pi

import matplotlib.patches
import matplotlib.pyplot as plt
import matplotlib.transforms
from shapely.geometry import *

# from Sumo import Plotting as splt
import Paths as params
from CustomSumoNetVis import Utils

# from PathPlanningTools import Base

DEFAULT_LANE_WIDTH = 3.2
US_MARKINGS = False  # if True, US-style lane markings will be drawn
USE_RGBA = True

def get_colour(obj, alpha=1, rgba=USE_RGBA, is_markings=False):
    if is_markings:
        if rgba:
            return (1, 1, 1, alpha)
        else:
            return "#FFFFFF"
    if isinstance(obj, Lane):
        if obj.allow == "pedestrian":
            if rgba:
                return (128 / 255, 128 / 255, 128 / 255, alpha)
            else:
                return "#808080"
        if obj.allow == "bicycle":
            if rgba:
                return (192 / 255, 66 / 255, 44 / 255, alpha)
            else:
                return "#C0422C"
        if obj.allow == "ship":
            if rgba:
                return (150 / 255, 200 / 255, 128 / 255, alpha)
            else:
                return "#96C8C8"
        if obj.allow == "authority":
            if rgba:
                return (255 / 255, 0 / 255, 0 / 255, alpha)
            return "#FF0000"
        if obj.disallow == "all":
            if rgba:
                return (1, 1, 1, alpha)
            else:
                return "#FFFFFF"
        if "passenger" in obj.disallow or "passenger" not in obj.allow:
            if rgba:
                return (92 / 255, 92 / 255, 92 / 255, alpha)
            else:
                return "#5C5C5C"
        else:
            if rgba:
                return (0, 0, 0, alpha)
            else:
                return "#000000"
    elif isinstance(obj, Junction):
        if rgba:
            return (92 / 255, 92 / 255, 92 / 255, alpha)
        else:
            return "#5C5C5C"


class Edge:
    def __init__(self, attrib):
        """
        Initializes an Edge object.
        :param attrib: dict of Edge attributes
        :type attrib: dict
        """
        self.id = attrib["id"]
        self._from_junction_str = attrib["from"] if "from" in attrib else ""
        self._to_junction_str = attrib["to"] if "to" in attrib else ""
        self.function = attrib["function"] if "function" in attrib else ""
        self.lanes = []
        self.from_junction = None
        self.to_junction = None
        if "shape" in attrib:
            coords = [[float(coord) for coord in xy.split(",")] for xy in attrib["shape"].split(" ")]
            self.alignment = LineString(coords)
        else:
            self.alignment = None
        self.border_lane = None
        self.centre_line = None
        self.road_width = 0

    def append_lane(self, lane):
        """
        Makes the specified Lane a child of the Edge
        :param lane: child Lane object
        :return: None
        :type lane: Lane
        """
        self.lanes.append(lane)
        lane.parentEdge = self

    def lane_count(self):
        """
        Returns the number of lanes to which this Edge is a parent
        :return: lane count
        """
        return len(self.lanes)

    def intersects(self, other):
        """
        Checks if any lane in the edge intersects a specified geometry
        :param other: the geometry against which to check
        :return: True if any lane intersects other, else False
        :type other: BaseGeometry
        """
        for lane in self.lanes:
            if other.intersects(lane.shape):
                return True
        return False

    def plot(self, ax, markings=True, alpha=1, background_colour=None):
        """
        Plots the lane
        :param ax: matplotlib Axes object
        :return: None
        :type ax: plt.Axes
        """
        for lane in self.lanes:
            lane.plot_shape(ax, background_colour=background_colour)
            if markings:
                lane.plot_lane_markings(ax, alpha)

        # self.plot_id(ax)

    def get_lane_by_index(self, lane_index):
        """
        Gets the edges lane with the given index.

        :param lane_index: the lane index
        :type lane_index: int
        :return: The Lane with the specified index, None, if Lane does not exist
        :rtype: Lane
        """
        for lane in self.lanes:
            if lane.index == lane_index:
                return lane
        return None

    def get_lane_by_id(self, lane_id):
        """
        Gets the edges lane with the given ID.

        :param lane_id: the lane ID
        :type lane_id: str
        :return: The Lane with the specified ID, None, if Lane does not exist
        :rtype: Lane
        """
        for lane in self.lanes:
            if lane.id == lane_id:
                return lane
        return None

    def _get_junction_objects(self, net):
        """
        Gets the from_junction and to_junction objects from the current network. Also sets the shape if the edge has no
        specific shape.

        :param net: the current network
        :type net: Net
        """
        self.from_junction = net.get_junction_by_id(self._from_junction_str)
        self.to_junction = net.get_junction_by_id(self._to_junction_str)
        if self.from_junction and self.to_junction and not self.alignment:
            coords = [[self.from_junction.pos_x, self.from_junction.pos_y],
                      [self.to_junction.pos_x, self.to_junction.pos_y]]
            self.alignment = LineString(coords)

    def plot_alignment(self, ax, style='r-'):
        """
        Plots the centerline alignment of the edge
        :param ax: matplotlib Axes object
        :type ax: plt.Axes
        :param style: the plot-style of the alignment
        :type style: str
        """
        if self.alignment:
            x, y = zip(*self.alignment.coords)
            ax.plot(x, y, style, linewidth=.5)

    def plot_road_border(self, ax, border_style='b-', centreline_style='r-'):
        """
        Plots the centerline alignment of the edge
        :param ax: matplotlib Axes object
        :type ax: plt.Axes
        :param border_style: the plot-style of the alignment
        :type border_style: str
        """
        if self.border_lane:
            outer_border = self.border_lane.alignment.parallel_offset(self.border_lane.width / 2, side="right")
            x, y = zip(*outer_border.coords)
            ax.plot(x, y, border_style, linewidth=.5)

            x, y = zip(*self.centre_line.coords)
            ax.plot(x, y, centreline_style, linewidth=.5)

    def _get_road_border(self):
        """
        Gets the borders of the area, which is accessible by a car, of the given edge as LineString.
        """
        border_lane = None
        for i in range(self.lane_count() - 1, -1, -1):
            disallowed = self.lanes[i].disallow
            allowed = self.lanes[i].allow
            if str.__eq__(allowed, '') and not str.__eq__(disallowed, 'all'):
                border_lane = self.lanes[i]
                self.road_width += border_lane.width
                continue
            else:
                break
        if border_lane is not None:
            self.border_lane = border_lane
            self.centre_line = self.border_lane.alignment.parallel_offset(self.road_width - self.border_lane.width / 2,
                                                                          side="left")

    def plot_id(self, ax):
        if self.from_junction and self.to_junction:

            delta_x = self.to_junction.pos_x - self.from_junction.pos_x
            delta_y = self.to_junction.pos_y - self.from_junction.pos_y

            node_x = self.from_junction.pos_x + delta_x / 2
            node_y = self.from_junction.pos_y + delta_y / 2

                # rect = matplotlib.patches.Rectangle((node_x - DEFAULT_LANE_WIDTH, node_y -
                #                                      DEFAULT_LANE_WIDTH), DEFAULT_LANE_WIDTH * 2, DEFAULT_LANE_WIDTH * 2,
                #                                     linewidth=1, color='w')
                # ax.add_patch(rect)
            ax.annotate(self.id, (node_x, node_y), size=DEFAULT_LANE_WIDTH * 2)
            # plt.text(node_x + 5, node_y + 5, str(self.id), fontsize=12)

    def get_length(self):
        if self.from_junction and self.to_junction:
            delta_x = self.to_junction.pos_x - self.from_junction.pos_x
            delta_y = self.to_junction.pos_y - self.from_junction.pos_y
        return ((delta_x) ** 2 + (delta_y) ** 2) ** .5
        # length = 0
        # if self.lane_count() > 0:
        #     for lane in self.lanes:
        #         length += lane.length
        # return length / self.lane_count()

    def plot_edge_border(self, ax, style='b-'):
        edge_width = - self.lanes[-1].width / 2
        for lane in self.lanes:
            edge_width += lane.width

        outer_border = self.lanes[-1].alignment.parallel_offset(edge_width, side="right")
        x, y = zip(*outer_border.coords)
        ax.plot(x, y, style, linewidth=.5)

    def get_green_stripe(self):
        # tree_diameter = 1.5 * DEFAULT_LANE_WIDTH
        # define green stripe:
        edge_width = - self.lanes[-1].width / 2 + DEFAULT_LANE_WIDTH / 4
        for lane in self.lanes:
            edge_width += lane.width

        border1 = self.lanes[-1].alignment.parallel_offset(edge_width, side="right")
        border2 = border1.parallel_offset(DEFAULT_LANE_WIDTH / 2, side="left")
        # x, y = zip(*border1.coords)
        # ax.plot(x, y, 'g-', linewidth=.5)
        # x, y = zip(*border2.coords)
        # ax.plot(x, y, 'g-', linewidth=.5)

        coordinates = []

        for pt in border1.coords:
            coordinates.append(pt)
        for pt in reversed(border2.coords):
            coordinates.append(pt)

        # line = LineString(coordinates)
        # x, y = zip(*line.coords)
        # ax.plot(x, y, 'r-', linewidth=.5)
        return Polygon(coordinates)






class Lane:
    def __init__(self, attrib):
        """
        Initialize a Lane object.
        :param attrib: dict of all of the lane attributes
        :type attrib: dict
        """
        self.id = attrib["id"]
        self.index = int(attrib["index"])
        self.speed = float(attrib["speed"])
        self.allow = attrib["allow"] if "allow" in attrib else ""
        self.disallow = attrib["disallow"] if "disallow" in attrib else ""
        self.width = float(attrib["width"]) if "width" in attrib else DEFAULT_LANE_WIDTH
        self.endOffset = attrib["endOffset"] if "endOffset" in attrib else 0
        self.acceleration = attrib["acceleration"] if "acceleration" in attrib else "False"
        coords = [[float(coord) for coord in xy.split(",")] for xy in attrib["shape"].split(" ")]
        self.alignment = LineString(coords)
        self.shape = self.alignment.buffer(self.width / 2, cap_style=CAP_STYLE.flat)
        self.length = float(attrib["length"]) if "length" in attrib else 0.0
        self.color = get_colour(self)
        self.parentEdge = None
        self.centre = None
        self.direction = self.get_lane_direction()

    def lane_color(self):
        """
        Returns the Sumo-GUI default lane color for this lane.
        :return: lane color
        """
        if self.allow == "pedestrian":
            return "#808080"
        if self.allow == "bicycle":
            return "#C0422C"
        if self.allow == "ship":
            return "#96C8C8"
        if self.allow == "authority":
            return "#FF0000"
        if self.disallow == "all":
            return "#FFFFFF"
        if "passenger" in self.disallow or "passenger" not in self.allow:
            return "#5C5C5C"
        else:
            return "#000000"

    def plot_alignment(self, ax, style='r-', alpha=1):
        """
        Plots the centerline alignment of the lane
        :param ax: matplotlib Axes object
        :return: None
        :type ax: plt.Axes
        """
        x, y = zip(*self.alignment.coords)
        ax.plot(x, y, style)

    def plot_shape(self, ax, alpha=1, background_colour=None):
        """
        Plots the entire shape of the lane
        :param ax: matplotlib Axes object
        :return: None
        :type ax: plt.Axes
        """
        if self.disallow == "all" and background_colour:
            colour = background_colour
        else:
            colour = get_colour(self, alpha)
        poly = matplotlib.patches.Polygon(self.shape.boundary.coords, True, color=colour, zorder=0)
        ax.add_patch(poly)

    def inverse_lane_index(self):
        """
        Returns the inverted lane index (i.e. counting from inside out)
        :return: inverted lane index
        """
        return self.parentEdge.lane_count() - self.index - 1

    def plot_lane_markings(self, ax, alpha):
        """
        Guesses and plots some simple lane markings. TODO: use fill_between to plot thickness in data coordinates
        :param ax: matplotlib Axes object
        :return: None
        :type ax: plt.Axes
        """
        if "passenger" in self.allow or "passenger" not in self.disallow and self.parentEdge.function != "internal":
            if self.inverse_lane_index() == 0:
                color, dashes = ("y", (1, 0)) if US_MARKINGS is True else ("w", (1, 0))
            else:
                color, dashes = "w", (3, 9)
            leftEdge = self.alignment.parallel_offset(self.width / 2, side="left")
            try:
                x, y = zip(*leftEdge.coords)
                line = Utils.LineDataUnits(x, y, linewidth=0.5, color=get_colour(self, is_markings=True, alpha=alpha),
                                           dashes=dashes)
                ax.add_line(line)
            except NotImplementedError:
                print("Can't print center stripe for lane " + self.id)

    def plot_alignment_offset(self, ax, offset=DEFAULT_LANE_WIDTH / 2, side="right"):
        """
        Plots the lanes alignment with the given offset.

        :param ax: Plot data series
        :type ax: plt.Axes
        :param offset: Offset to the original lane alignment
        :type offset: float
        :param side: specifies on which side of the main alignment the offset will be drawn
        :type side: str
        """
        outer_border = self.alignment.parallel_offset(offset, side=side)
        x, y = zip(*outer_border.coords)
        ax.plot(x, y)

    def get_stopping_point(self):
        return self.alignment.coords[-1]

    def get_starting_point(self):
        return self.alignment.coords[0]

    def get_lane_as_node(self):
        pt1 = self.alignment.coords[0]
        pt2 = self.alignment.coords[-1]
        delta_x = pt2[0] - pt1[0]
        delta_y = pt2[1] - pt1[1]

        node_x = pt1[0] + delta_x / 2
        node_y = pt1[1] + delta_y / 2

        self.centre = (node_x, node_y)

    def get_lane_direction(self):
        """
        Calculates the direction of the current lane from start to end alignment point

        :return: direction between start and end of the lane alignment coordinates
        :rtype: float
        """
        origin_x = self.alignment.coords[0][0]
        origin_y = self.alignment.coords[0][1]
        dest_x = self.alignment.coords[-1][0]
        dest_y = self.alignment.coords[-1][1]
        return atan2((dest_y - origin_y), (dest_x - origin_x))


class Junction:
    def __init__(self, attrib):
        """
        Initializes a Junction object.
        :param attrib: dict of junction attributes.
        :type attrib: dict
        """
        self.pos_x = float(attrib["x"])
        self.pos_y = float(attrib["y"])
        self.id = attrib["id"]
        self.shape = None
        self.color = get_colour(self)
        if "shape" in attrib:
            coords = [[float(coord) for coord in xy.split(",")] for xy in attrib["shape"].split(" ")]
            if len(coords) > 2:
                self.shape = Polygon(coords)
        if "incLanes" in attrib:
            self.incomings = [str(incoming) for incoming in attrib["incLanes"].split(" ")]
        else:
            self.incomings = None
        self.incoming_edges = []

    def plot(self, ax, alpha=1):
        """
        Plots the Junction.
        :param ax: matplotlib Axes object
        :return: None
        :type ax: plt.Axes
        """
        if self.shape is not None:
            poly = matplotlib.patches.Polygon(self.shape.boundary.coords, True, color=get_colour(self, alpha), zorder=0)
            ax.add_patch(poly)

    def get_incoming_edges(self, network):
        """
        Gets all the incoming edges for the junction, based on the network.

        :param network: the current network
        :type network: Net
        """
        for incoming_lane in self.incomings:
            inc_edge = network.get_edge_by_id(incoming_lane[:-2])
            if inc_edge:
                self.incoming_edges.append(inc_edge)


class Net:
    def __init__(self, file):
        """
        Initializes a Net object from a Sumo network file
        :param file: path to Sumo network file
        :type file: str
        """
        self.edges = []
        self.junctions = []
        self.connections = []
        net = ET.parse(file).getroot()
        for obj in net:
            if obj.tag == "edge":
                edge = Edge(obj.attrib)
                for laneObj in obj:
                    lane = Lane(laneObj.attrib)
                    edge.append_lane(lane)
                self.edges.append(edge)
            elif obj.tag == "junction":
                junction = Junction(obj.attrib)
                self.junctions.append(junction)
            elif obj.tag == "connection":
                connection = Connection(obj.attrib, self)
                self.connections.append(connection)

        for edge in self.edges:
            edge._get_junction_objects(self)
            edge._get_road_border()

        for connection in self.connections:
            connection.junction = connection.from_edge.to_junction

        for junction in self.junctions:
            junction.get_incoming_edges(self)

    def _get_extents(self):
        lane_geoms = []
        for edge in self.edges:
            for lane in edge.lanes:
                lane_geoms.append(lane.shape)
        polygons = MultiPolygon(lane_geoms)
        return polygons.bounds

    def plot(self, ax=None, clip_to_limits=False, zoom_to_extents=True, plot_markings=True, alpha=1,
             include_internal=False, face_colour=None):
        """
        Plots the Net.
        :param ax: matplotlib Axes object. Defaults to current axes.
        :param clip_to_limits: if True, only objects in the current view will be drawn. Speeds up saving of animations.
        :param zoom_to_extents: if True, window will be set to the network extents. Ignored if clip_to_limits is True
        :param plot_markings: if True, markings will be added to the plot
        :return: None
        :type ax: plt.Axes
        :type clip_to_limits: bool
        :type zoom_to_extents: bool
        """
        if ax is None:
            ax = plt.gca()

        if face_colour:
            ax.set_facecolor(face_colour)

        if zoom_to_extents and not clip_to_limits:
            x_min, y_min, x_max, y_max = self._get_extents()
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
        ax.set_clip_box(ax.get_window_extent())
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()
        bounds = [[xmin, ymax], [xmax, ymax], [xmax, ymin], [xmin, ymin]]
        window = Polygon(bounds)
        for edge in self.edges:
            if not clip_to_limits or edge.intersects(window):
                if not include_internal and edge.function == "internal":
                    continue
                else:
                    edge.plot(ax, plot_markings, alpha, background_colour=face_colour)
        for junction in self.junctions:
            if not clip_to_limits or (junction.shape is not None and junction.shape.intersects(window)):
                junction.plot(ax, alpha)

    def plot_trees(self, ax, density=.5, tree_diameter=1.5 * DEFAULT_LANE_WIDTH):
        planting_area = []
        for edge in self.edges:
            if edge.function != 'internal':
                planting_area.append(edge.get_green_stripe())
                x,y = edge.get_green_stripe().exterior.xy
                ax.plot(x, y, 'g-')

        total_area = 0
        for poly in planting_area:
            total_area += poly.area

        single_tree_area = (1.5 * DEFAULT_LANE_WIDTH / 2) ** 2 * pi
        nbr_of_trees = int(density * total_area / single_tree_area)

        for i in range(0, nbr_of_trees):
            stripe = random.choice(planting_area)
            tree_pos = self._get_random_point_in_polygon(stripe)
            plot_random_tree(ax, tree_pos, tree_diameter)

    def _get_random_point_in_polygon(self, poly):
        minx, miny, maxx, maxy = poly.bounds
        while True:
            p = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
            if poly.contains(p):
                return (p.x, p.y)

    def get_edge_by_id(self, edge_id):
        """
        Gets the edge with the given id from the network.

        :param edge_id: The ID of the edge
        :type edge_id: str
        :return: Returns the edge with the given ID. If the edge does not exist, None is returned
        :rtype: Edge
        """
        for edge in self.edges:
            if str.__eq__(edge.id, edge_id):
                return edge
        return None

    def get_junction_by_id(self, junction_id):
        """
        Gets the junction with the given id from the network.

        :param junction_id: The ID of the junction
        :type junction_id: str
        :return: Returns the junction with the given ID. If the edge does not exist, None is returned
        :rtype: Junction
        """
        for junction in self.junctions:
            if str.__eq__(junction.id, junction_id):
                return junction
        return None

    def get_lane_object(self, edge_id, lane_index):
        """
        Gets a lane object based on the give edge id and the given lane index

        :param edge_id: the Id of the parent edge
        :type edge_id: str
        :param lane_index: lane index
        :type lane_index: int
        :return: returns the lane object if found, of not None
        :rtype: Lane
        """
        edge = self.get_edge_by_id(edge_id)
        if edge:
            return edge.get_lane_by_index(lane_index)

    def plot_borders(self, ax, edge_style="-", plot_junctions=False, include_internals=False, junction_style="ro"):
        """
        Plots the outlines of the network in order to define search spaces for path finding algorithms

        :param ax: plot data series
        :type ax: plt.Axes
        :param edge_style: plot style of the edge alignments
        :type edge_style: str
        :param junction_style: plot style of the junctions
        :type junction_style: str
        """
        for edge in self.edges:
            if not include_internals and str.__eq__(edge.function, "internal"):
                continue
            else:
                edge.plot_road_border(ax)
        if plot_junctions:
            for junction in self.junctions:
                ax.plot(junction.pos_x, junction.pos_y, junction_style)

    def plot_specific_connection(self, ax, origin_edge):
        """
        Plots a specific connection, taking the origin_edge and the outermost car-allowing lane as the origin. In case
        such a connection actually exists.

        :param ax: plot data series
        :type ax: plt.Axes
        :param origin_edge: the edge to be checked
        :type origin_edge: Edge
        """
        for connection in self.connections:
            if str.__eq__(connection.from_edge.id, origin_edge.id):
                if connection.via:
                    for lane in connection.via.lanes:
                        if origin_edge.border_lane:
                            if origin_edge.border_lane.index == connection.from_lane.index:
                                lane.plot_alignment_offset(ax)

    def plot_connection(self, origin, dest, ax=None):
        """
        Plots a connection between the origin and the destination edge, giving two LineStrings, which connect the border-
        lines of the drivable area, to be used for the rrt to do validation checks on the nodes.

        :param origin: origin edge
        :type origin: Edge
        :param dest: destinating edge
        :type dest: Edge
        :param ax: plot data-series, if None, nothing will be plotted
        :type ax: plt.Axes
        :return: Returns the two connecting LineStrings
        :rtype: (LineString, LineString)
        """
        connect_1, connect_2 = None, None
        for connection in self.connections:
            if connection.from_edge.id == connection.to_edge.id:
                continue
            elif str.__eq__(connection.from_edge.id, origin.id) and str.__eq__(connection.to_edge.id, dest.id):
                pt1_1 = origin.border_lane.alignment.parallel_offset(origin.border_lane.width / 2, side="right").coords[
                    0]
                pt2_1 = dest.border_lane.alignment.parallel_offset(dest.border_lane.width / 2, side="right").coords[-1]
                pt3_1 = origin.centre_line.coords[-1]
                pt4_1 = dest.centre_line.coords[0]
                junction_pt = (connection.junction.pos_x, connection.junction.pos_y)

                if str.__eq__(connection.dir, "l"):
                    connect_1 = LineString([pt3_1, pt4_1])
                    connect_2 = LineString([pt1_1, junction_pt, pt2_1])
                elif str.__eq__(connection.dir, "r"):
                    connect_1 = LineString([pt1_1, pt2_1])
                    connect_2 = LineString([pt3_1, junction_pt, pt4_1])
                elif str.__eq__(connection.dir, "s"):
                    connect_1 = LineString([pt1_1, pt2_1])
                    connect_2 = LineString([pt3_1, pt4_1])

                if ax:
                    x, y = zip(*connect_1.coords)
                    ax.plot(x, y, 'y--', linewidth=.8)
                    x, y = zip(*connect_2.coords)
                    ax.plot(x, y, 'y--', linewidth=.8)

        return connect_1, connect_2

    def add_missing_connections(self):
        """
        Adds missing connections to the network, which are defined by going from one lane to another connected to the
        same parent edge. This enables lane-changing within the A* Path Planning Algorithm.

        """
        for edge in self.edges:
            drivable_lanes = []
            for lane in edge.lanes:
                if str.__eq__(lane.allow, '') and not str.__eq__(lane.disallow, 'all'):
                    drivable_lanes.append(lane)

            for d_lane in drivable_lanes:
                for c_lane in drivable_lanes:
                    if d_lane.id != c_lane.id:
                        int_connect = Connection(None, None)
                        int_connect._make_intern_connection(edge, d_lane, c_lane)
                        self.connections.append(int_connect)

    def get_connection(self, start_lane, end_lane):
        start_edge_id = start_lane.id[:-2]
        end_edge_id = end_lane.id[:-2]

        for connection in self.connections:
            if connection.from_edge.id == start_edge_id and connection.to_edge.id == end_edge_id:
                if start_lane.index == connection.from_lane.index and end_lane.index == connection.to_lane.index:
                    return connection


    def get_lane_by_id(self, lane_id):
        """
        Gets the lane based on the given ID

        :param lane_id: the lane ID to look for
        :type lane_id: str
        :return: the lane, if found
        :rtype: Lane
        """
        found_lane = None
        for edge in self.edges:
            for lane in edge.lanes:
                found_lane = edge.get_lane_by_id(lane_id)
                if found_lane: return found_lane
        return found_lane

    def get_borders(self, path, ax=None):
        """
        Highlights the outlines of a given path (car accessible area only) and returns the borders

        :param path: path to be highlighted
        :type path: list[Lane]
        :param ax: plot data series, if None, nothing is plotted and only the borders will be returned
        :type ax: plt.Axes
        :return: List of all the borders of the current drivable area of the path
        :rtype: list[LineString]
        """
        borders = []
        connection_borders = dict()
        for i in range(0, len(path) - 1):
            if ax:
                path[i].parentEdge.plot_road_border(ax, border_style='b-')
            connect1, connect2 = self.plot_connection(path[i].parentEdge, path[i + 1].parentEdge, ax)
            if connect1 and connect2:
                borders.append(connect1)
                borders.append(connect2)
                connection = self.get_connection(path[i], path[i + 1])
                connection_borders[connection._via_str] = connect2

            outer_border = path[i].parentEdge.border_lane.alignment.parallel_offset(
                path[i].parentEdge.border_lane.width / 2, side="right")
            borders.append(outer_border)
            borders.append(path[i].parentEdge.centre_line)
        if ax:
            path[-1].parentEdge.plot_road_border(ax, border_style='b-', centreline_style='r-')
        outer_border = path[-1].parentEdge.border_lane.alignment.parallel_offset(
            path[-1].parentEdge.border_lane.width / 2, side="right")
        borders.append(outer_border)
        borders.append(path[-1].parentEdge.centre_line)

        return borders, connection_borders



class Connection:
    def __init__(self, attrib, network):
        """
        Initializes a Connection object.

        :param attrib: dict of connection attributes.
        :type attrib: dict
        :param network: The current network
        :type network: Net
        """
        if attrib:
            self._from_edge_str = attrib["from"] if "from" in attrib else ""
            self._to_edge_str = attrib["to"] if "to" in attrib else ""
            self._from_lane_str = int(attrib["fromLane"]) if "fromLane" in attrib else ""
            self._to_lane_str = int(attrib["toLane"]) if "toLane" in attrib else ""
            self._via_str = attrib["via"] if "via" in attrib else ""
            self.dir = attrib["dir"] if "dir" in attrib else ""
            if "shape" in attrib:
                coords = [[float(coord) for coord in xy.split(",")] for xy in attrib["shape"].split(" ")]
                self.shape = LineString(coords)
        else:
            self.shape = None
        self.from_edge = None
        self.to_edge = None
        self.from_lane = None
        self.to_lane = None
        self.via = None
        self.junction = None

        if network:
            self._get_connection_objects(network)

    def _make_intern_connection(self, edge, from_lane, to_lane):
        """
        Creates a connection object, going from one lane to another lane within the same edge

        :param edge: Parent edge
        :type edge: Edge
        :param from_lane: departing lane
        :type from_lane: Lane
        :param to_lane: arriving lane
        :type to_lane: Lane
        """
        self.from_edge = edge
        self.to_edge = edge
        self.from_lane = from_lane
        self.to_lane = to_lane
        self.dir = "="

    def _get_connection_objects(self, network):
        """
        Gets the actual objects for the connection.

        :param network: The current network
        :type network: Net
        """
        self.from_edge = network.get_edge_by_id(self._from_edge_str)
        self.to_edge = network.get_edge_by_id(self._to_edge_str)
        self.via = network.get_lane_by_id(self._via_str)
        self.from_lane = self.from_edge.get_lane_by_index(self._from_lane_str)
        self.to_lane = self.to_edge.get_lane_by_index(self._to_lane_str)

    def plot_alignment(self, ax, style="b--"):
        """
        Plots the shape of the connection if one is given

        :param ax: plot data series
        :type ax: plt.Axes
        :param style: the plot style of the shape
        :type style: str
        """
        if self.via:
            self.via.plot_alignment(ax)

    def get_connection_as_edge(self):
        # if self.from_edge.function != 'internal' and self.to_edge.function != 'internal':
        if self.from_lane and self.to_lane:
            start_x, start_y = (self.from_lane.centre[0], self.from_lane.centre[1])
            end_x, end_y = (self.to_lane.centre[0], self.to_lane.centre[1])
            return (start_x, start_y, end_x, end_y)
        else:
            return None

def plot_random_tree(ax, pos, diameter):
    tree_nbr = random.randint(1, 4)
    if tree_nbr == 1:
        image = plt.imread(params.IMG_PATHS().tree1)
    if tree_nbr == 2:
        image = plt.imread(params.IMG_PATHS().tree2)
    if tree_nbr == 3:
        image = plt.imread(params.IMG_PATHS().tree3)
    else:
        image = plt.imread(params.IMG_PATHS().tree4)

    img = ax.imshow(image, origin="lower", zorder=12)

    scale = diameter / image.shape[0]
    tscale = matplotlib.transforms.Affine2D().scale(scale)

    # tx1 = diameter / 2
    # ty1 = diameter / 2
    # ttranslate1 = matplotlib.transforms.Affine2D().translate(-tx1, -ty1)

    # angle = random.randint(0, 180)
    # tr = matplotlib.transforms.Affine2D().rotate_deg(angle)

    tx = pos[0] - diameter / 2
    ty = pos[1]- diameter / 2
    ttranslate2 = matplotlib.transforms.Affine2D().translate(tx, ty)

    trans_data = tscale + ttranslate2 + ax.transData
    img.set_transform(trans_data)
    return img

if __name__ == "__main__":
    net = Net('../MT_Course.net.xml')
    fig, ax = plt.subplots()
    net.plot(ax)
    plt.show()
