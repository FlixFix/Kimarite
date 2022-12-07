import copy
import random
from math import cos, degrees, pi, radians, sin

import matplotlib.patches
import matplotlib.pyplot as plt
import matplotlib.transforms
import numpy as np
import traci
from matplotlib.path import Path
from shapely.geometry import LineString, Point, Polygon

import Paths as params
import Sumo.SumoHelperFcts as shf
import Sumo.SumoParameters as sp
from PathPlanningTools import Base


def plot_marker(pos, direction, speed, ax, size=.5):
    """
    Plots a fancy marker with the correct colour and direction to the given position.

    :param pos: the position of the marker
    :type pos: Point
    :param direction: the direction of the marker
    :type direction: float
    :param speed: the respective speed
    :type speed: float
    :param ax: the given plot data series
    :type ax: plt.Axes
    :param size: the marker size
    :type size: float
    """
    image = plt.imread(get_marker(speed))
    img = ax.imshow(image, origin="lower", zorder=9)

    scale = sp.CAR_WIDTH * size / image.shape[0]
    tscale = matplotlib.transforms.Affine2D().scale(scale)

    tx1 = scale * image.shape[1] / 2
    ty1 = scale * image.shape[0] / 2
    ttranslate1 = matplotlib.transforms.Affine2D().translate(-tx1, -ty1)

    tr = matplotlib.transforms.Affine2D().rotate_deg(degrees(direction))

    tx = pos.x
    ty = pos.y
    ttranslate2 = matplotlib.transforms.Affine2D().translate(tx, ty)

    trans_data = tscale + ttranslate1 + tr + ttranslate2 + ax.transData
    img.set_transform(trans_data)


def get_marker(speed):
    """
    Gets the marker based on the current speed.

    :param speed: the current speed
    :type speed: float
    :return: the respective marker image path
    :rtype: str
    """
    if speed < 2.78:
        return params.IMG_PATHS().marker_low
    elif 2.78 <= speed <= 8.33:
        return params.IMG_PATHS().marker_low_to_medium
    elif 8.33 < speed <= 13.9:
        return params.IMG_PATHS().marker_medium
    elif 13.9 < speed <= 22.22:
        return params.IMG_PATHS().marker_medium_to_high
    else:
        return params.IMG_PATHS().marker_high


def plot_field_of_view(fov, ax, clr=sp.Colors().field_of_view):
    """
    Plots the given field of view, using a fancy plotting

    :param fov: the field of view to be plotted
    :type fov: Base.FieldOfView
    :param ax: the given plot data series
    :type ax: plt.Axes
    :param clr: the colour of the field of view plot
    :type clr: sp.Colors
    """
    # delete old stuff
    for patch in fov.fancy_patches:
        if patch in ax.patches:
            ax.patches.remove(patch)

    turning_angle = fov.direction + fov.angle / 2
    delta_phi = fov.angle / (100 + 1)
    d_reach = int(fov.reach / 100)

    patches = []
    for i in range(0, fov.reach, d_reach):
        patch_points = []
        for j in range(0, 102):
            x_i = fov.origin[0] + i * sin(radians(j * delta_phi - turning_angle + 90))
            y_i = fov.origin[1] + i * cos(radians(j * delta_phi - turning_angle + 90))
            patch_points.append((x_i, y_i))
        second_set = []
        for j in range(0, 102):
            x_i = fov.origin[0] + (i + d_reach) * sin(radians(j * delta_phi - turning_angle + 90))
            y_i = fov.origin[1] + (i + d_reach) * cos(radians(j * delta_phi - turning_angle + 90))
            second_set.append((x_i, y_i))
        second_set.reverse()
        patch_points.extend(second_set)
        patches.append(patch_points)

    delta_alpha = (clr[3] - .1) / 100
    for i in range(0, len(patches)):
        alpha = clr[3] - i * delta_alpha
        current_clr = (clr[0], clr[1], clr[2], alpha)
        path = Path(patches[i])
        plt_patch = matplotlib.patches.PathPatch(path, facecolor=current_clr, lw=0, zorder=7)
        ax.add_patch(plt_patch)
        fov.fancy_patches.append(plt_patch)


def plot_cyclist(id, ax, type="default", alpha=1):
    """
    Plots a SUMO cyclist to a given plot data series with the respective colour.

    :param id: id of the cyclist to be plotted
    :type id: str
    :param ax: the given plot data series
    :type ax: plt.Axes
    :param type: type of the agent
    :type type: str
    :return: the plotted image
    :rtype: matplotlib.image.AxesImage
    """
    if type == "default" or type == "minor":
        image = plt.imread(params.IMG_PATHS().default_cyclist)
    elif type == "conflict" or type == "oncoming":
        image = plt.imread(params.IMG_PATHS().conflict_bike)
    elif type == "critical":
        image = plt.imread(params.IMG_PATHS().critical_bike)
    elif type == "yield":
        image = plt.imread(params.IMG_PATHS().yield_bike)

    img = ax.imshow(image, origin="lower", zorder=10, alpha=alpha)


    scale = traci.vehicle.getWidth(id) / image.shape[0]
    tscale = matplotlib.transforms.Affine2D().scale(scale)

    tx1 = traci.vehicle.getLength(id) / 2
    ty1 = traci.vehicle.getWidth(id) / 2
    ttranslate1 = matplotlib.transforms.Affine2D().translate(-tx1, -ty1)

    tr = matplotlib.transforms.Affine2D().rotate_deg(shf.get_plt_angle(traci.vehicle.getAngle(id)))

    tx = traci.vehicle.getPosition(id)[0]
    ty = traci.vehicle.getPosition(id)[1]
    ttranslate2 = matplotlib.transforms.Affine2D().translate(tx, ty)

    trans_data = tscale + ttranslate1 + tr + ttranslate2 + ax.transData
    img.set_transform(trans_data)
    return img


def plot_sumo_agents(ego_id, critical_id, conflict_id_list, ax, plot_all=False):
    """
    Plots all the SUMO agents from the current situation

    :param ego_id: ID of the autonomous agent
    :type ego_id: str
    :param critical_id: ID of the critical agent
    :type critical_id: str
    :param conflict_id_list: list of all the conflicting agents of the scenario
    :type conflict_id_list: list[str]
    :param ax: the given plot data series
    :type ax: plt.Axes
    :param plot_all: plots all the other agents of the Scenario too
    :type plot_all: bool
    """
    if critical_id:
        plot_cyclist(critical_id, ax, type="critical")

    if len(conflict_id_list) > 0:
        for veh in conflict_id_list:
            if veh != critical_id:
                plot_cyclist(veh, ax, type="conflict")

    if plot_all:
        for id in traci.vehicle.getIDList():
            if id != ego_id and id not in conflict_id_list and id != critical_id:
                plot_cyclist(id, ax)


def plot_trajectory(trajectory, cx, style='o-', fancy=True):
    """
    Plots the given trajectory using the speed gradient

    :param trajectory: the given trajectory
    :type trajectory: list[Points]
    :param speeds: the respective speeds for the trajectory
    :type speeds: list[float]
    :param cx: the given plot data series
    :type cx: plt.Axes
    :param style: the marker style for the trajectory data points
    :type style: str
    """
    if fancy:
        for i in range(0, len(trajectory)):
            pos = Point([trajectory[i, 2], trajectory[i, 3]])
            plot_marker(pos, radians(shf.get_plt_angle(trajectory[i, 5])), trajectory[i, 4], cx)


def plot_configuration_rect(veh, ax, delta=.5):
    """
    Plots a rectangle around a given sumo vehicle in a red colour and red hatching.

    :param veh: the ID of the sumo vehicle
    :type veh: str
    :param ax: the given plot data series
    :type ax: plt.Axes
    """
    rect = shf.get_sumo_vehicle_configuration_poly(veh, delta=delta)
    path_points = []
    for pt in rect.boundary.coords:
        path_points.append((pt[0], pt[1]))
    path = Path(path_points)

    hatch_color = (sp.Colors().blocked_space[0], sp.Colors().blocked_space[1], sp.Colors().blocked_space[2], 1)

    plt_patch = matplotlib.patches.PathPatch(path, facecolor=sp.Colors().blocked_space, fill=True, edgecolor=hatch_color,
                                             lw=1, hatch='\\', zorder=3)
    ax.add_patch(plt_patch)


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

    tx1 = diameter / 2
    ty1 = diameter / 2
    ttranslate1 = matplotlib.transforms.Affine2D().translate(-tx1, -ty1)

    angle = random.randint(0, 180)
    tr = matplotlib.transforms.Affine2D().rotate_deg(angle)

    tx = pos[0]
    ty = pos[1]
    ttranslate2 = matplotlib.transforms.Affine2D().translate(tx, ty)

    trans_data = tscale + ttranslate1 + tr + ttranslate2 + ax.transData
    img.set_transform(trans_data)
    return img


def show_values_on_bars(axs, h_v="v", space=0.4):
    def _show_on_single_plot(ax):
        if h_v == "v":
            for p in ax.patches:
                _x = p.get_x() + p.get_width() / 2
                _y = p.get_y() + p.get_height()
                value = int(p.get_height())
                ax.text(_x, _y, value, ha="center", va="center")
        elif h_v == "h":
            for p in ax.patches:
                _x = p.get_x() + p.get_width() + float(space)
                _y = p.get_y() + p.get_height() / 2
                value = str(round(p.get_width(), 4))
                ax.text(_x, _y, value + " %", ha="left", va="center")

    if isinstance(axs, np.ndarray):
        for idx, ax in np.ndenumerate(axs):
            _show_on_single_plot(ax)
    else:
        _show_on_single_plot(axs)


class RRTScene:
    def __init__(self, parent):
        self.data_monitor = parent
        self.rrt_tree = None
        self.target_node = None
        self.connection = None
        self.conflict_agent_pos = None
        self.conflict_agent_angle = None
        self.path = None
        self.configuration_space = None
        self.agent = None


    def create_scene(self, rrt_tree, conflict_veh, conflict_angle, path, configuration_space, target_node,
                     connection, agent):
        """
        Creates a rrt* scene for later plotting.

        :param rrt_tree: the respective rrt tree
        :type rrt_tree: Base.Tree
        :param conflict_veh: the ID of the conflicting vehicle
        :type conflict_veh: str
        :param conflict_angle: the angle of the conflicting vehicle at that time
        :type conflict_angle: float
        :param path: the chosen dubins path based on the rrt planning
        :type path: LineString
        :param configuration_space: the configuration space / junction area for the rrt planning
        :type configuration_space: Polygon
        :param target_node: the target node of the planning (last node of junction via alignment)
        :type target_node: Base.Node
        :param connection: the respective coonection
        :type connection: Net.Connection
        :param agent: the ego agent
        :type agent: Base.Agent
        """
        self.rrt_tree = copy.deepcopy(rrt_tree)
        self.target_node = copy.deepcopy(target_node)
        self.conflict_agent_pos = conflict_veh
        self.conflict_agent_angle = radians(shf.get_plt_angle(conflict_angle))
        self.path = path
        self.configuration_space = configuration_space
        self.connection = connection
        self.agent = copy.deepcopy(agent)

    def plot(self):
        """
        Plots the RRT Scene into a new figure.

        """
        fig = plt.figure()
        ax = plt.subplot(111)
        plt.axis('equal')

        junction_colour = (92 / 255, 92 / 255, 92 / 255, .6)
        poly = matplotlib.patches.Polygon(self.configuration_space.exterior.coords, True,
                                          color=junction_colour, zorder=0)
        ax.add_patch(poly)

        x_min, y_min, x_max, y_max = self.connection.junction.shape.boundary.bounds
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

        self.rrt_tree.plot_target_path(self.target_node, ax)
        self.rrt_tree.plot(ax, style='w-')

        # create dummy agent:
        conflict_pos = Base.Node(self.conflict_agent_pos[0], self.conflict_agent_pos[1], self.conflict_agent_angle)
        traci.route.add("dummy_route", ["gneE56", "gneE61"])
        shf.add_sumo_bike(conflict_pos, "dummy_route", "gneE61", 2, "dummy")
        traci.simulationStep()

        plot_configuration_rect("dummy", ax)
        plot_cyclist("dummy", ax, "critical")
        plt.pause(.1)
        self.agent.plot(ax, include_vision_field=False, include_sensor=False)
        plt.pause(.1)
        self.data_monitor.control_unit.plotter.plot_path(ax, path=self.path)


if __name__ == "__main__":
    fig = plt.figure()
    ax = plt.subplot(111)

    # pos = [(50, 50), (80,80)]
    # dir = (0, pi/4, pi)
    #
    # for i in range(0, 2):
    #     plot_marker(pos[i], dir[i], ax)

    start_point = Base.Node(0, 0, pi)
    ego_agent = Base.Agent(start_point, sp.CAR_LENGTH, sp.CAR_WIDTH, sp.MIN_CURVATURE)
    fov = Base.FieldOfView(ego_agent, 100, 130, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2))

    plt.axis('equal')
    plt.xlim(-100, 100)
    plt.ylim(-100, 100)
    # for ele in patches:
    #     plot_patch(ele, ax)
    #     plt.pause(.1)

    patches = plot_field_of_view(fov, ax)
    plt.show()
    print("test")