import os
import sys
import uuid
from math import inf, isnan, pi, radians

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import traci
from shapely import affinity
from shapely.geometry import Point, Polygon

import CustomSumoNetVis
import HelperFcts as hf
import Paths as params
from PathPlanningTools import Base
from Sumo import SumoParameters as sp

# ------------------------ DEFINES ----------------------
color = hf.Colours()
images = params.IMG_PATHS()
network_settings = params.NETWORK("ValidationNetwork.net.xml")
paths = params.SUMO_PATHS()
# -------------------------------------------------------


def get_sumo_angle(angle):
    """
    Sets orientation of Sumo-Vehicle from simulation angle.

    :param angle: Simulation angle (CCW from East) [rad]
    :return: Sumo angle
    :type: angle: float
    """
    return (450 - angle * 180 / pi) % 360


def get_plt_angle(angle):
    """
    Sets orientation of Sumo-Vehicle from simulation angle.

    :param angle: SUMO Angle (CW from North) [deg]
    :return: Plt angle in degrees
    :type: angle: float
    """
    return (450 - angle) % 360


def transform_to_origin(conflict_veh, ego_agent):
    """
    Transforms the current situation between the ego agent and the conflict agent to origin for better conflict checks.

    :param conflict_veh: the ID of the conflict vehicle
    :type conflict_veh: str
    :param ego_agent: the ego agent
    :type ego_agent: Base.Agent
    :return: the transformed conflict position and the transformed conflict angle
    :rtype: tuple[tuple[float], float]
    """
    conflict_pos = traci.vehicle.getPosition(conflict_veh)
    ego_pos = (ego_agent.position.x, ego_agent.position.y)
    ego_angle = ego_agent.angle
    conflict_pos = (conflict_pos[0] - ego_pos[0], conflict_pos[1] - ego_pos[1])
    conflict_pos = hf.rotate([0, 0], [conflict_pos[0], conflict_pos[1]], -ego_angle)
    conflict_angle = radians(get_plt_angle(traci.vehicle.getAngle(conflict_veh))) - ego_angle
    if conflict_angle < 0:
        conflict_angle += 2 * pi
    return conflict_pos, conflict_angle


def start_sumo():
    """
    This function starts SUMO.
    """
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    sumo_cmd = [paths.exe, "-c", paths.config_file, "--start"]
    traci.start(sumo_cmd)


def get_path_points(path):
    px = []
    py = []
    yaw =[]
    for ele in path:
        for i in range(0, len(ele.segment[0])):
            px.append(ele.segment[0][i])
            py.append(ele.segment[1][i])
            yaw.append(ele.segment[2][i])
    return px, py, yaw


def create_agent_from_trajectory_data(path_to_data, id="tum_cyclist"):
    """
    Creates an agent based on the trajectory data gathered by TUM's simulator studies.

    :param path_to_data: path to the trajectory data
    :type path_to_data: str
    :param id: anticipated ID of the agent
    :type id: str
    :return: the created agent
    :rtype: Base.Agent
    """
    df = pd.read_csv(path_to_data, usecols=['time', 'sim_time', 'TESIS_0_x', 'TESIS_0_y',
                                            'TESIS_0_speed', 'TESIS_0_angle'])

    agent = Base.Agent(autonomous=False, sumo_data=df.values, type="bike")
    agent.id = id
    return agent


def add_sumo_bike(pos, route, start_edge_id, start_lane_nbr, id):
    """
    Adds a bike agent to the current SUMO Simulation.

    :param pos: start position of the agent
    :type pos: Base.Node
    :param route: the route id to which the agent will be assigned
    :type route: str
    :param start_edge_id: the id of the starting edge
    :type start_edge_id: str
    :param start_lane_nbr: the number of the starting lane
    :type start_lane_nbr: int
    :param id: the ID of the newly added agent
    :type id: str
    """
    traci.vehicle.add(id, departPos=0,
                      departSpeed=sp.CONFLICT_START_SPEED, routeID=route)
    traci.vehicle.setType(id, "bike")
    traci.vehicle.setVehicleClass(id, "bicycle")
    traci.vehicle.moveToXY(id, lane=start_lane_nbr, edgeID=start_edge_id, x=pos.x, y=pos.y, angle=get_sumo_angle(pos.direction), keepRoute=2)
    traci.vehicle.setSpeedMode(id, 0)
    traci.vehicle.setSpeed(id, sp.CONFLICT_START_SPEED)
    agent = Base.Agent(pos, type="sumo", length=traci.vehicle.getLength(id), width=traci.vehicle.getWidth(id))
    agent.id = id
    return agent


def get_sumo_vehicle_as_poly(veh_id):
    """
    Returns a SUMO vehicle as a Polygon

    :param veh_id: the ID of the vehicle
    :type veh_id: str
    :return: The bounding box of the vehicle
    :rtype: Polygon
    """
    pos = traci.vehicle.getPosition(veh_id)
    angle = get_plt_angle(traci.vehicle.getAngle(veh_id))

    pt1 = Point(pos[0] - sp.CAR_LENGTH / 2, pos[1] + sp.CAR_WIDTH / 2)
    pt2 = Point(pos[0] + sp.CAR_LENGTH / 2, pos[1] + sp.CAR_WIDTH / 2)
    pt3 = Point(pos[0] + sp.CAR_LENGTH / 2, pos[1] - sp.CAR_WIDTH / 2)
    pt4 = Point(pos[0] - sp.CAR_LENGTH / 2, pos[1] - sp.CAR_WIDTH / 2)
    poly = Polygon([[p.x, p.y] for p in [pt1, pt2, pt3, pt4]])
    return affinity.rotate(poly, angle, 'center')


def get_sumo_vehicle_configuration_poly(veh_id, delta=0):
    """
    Returns a SUMO vehicle including the configuration space as a Polygon

    :param veh_id: the ID of the vehicle
    :type veh_id: str
    :return: The bounding box of the vehicle
    :rtype: Polygon
    """
    pos = traci.vehicle.getPosition(veh_id)
    angle = get_plt_angle(traci.vehicle.getAngle(veh_id))

    pt1 = Point(pos[0] - sp.CAR_LENGTH / 2 - sp.PASSING_DIST + delta, pos[1] + sp.CAR_WIDTH / 2 + sp.PASSING_DIST - delta)
    pt2 = Point(pos[0] + sp.CAR_LENGTH / 2 + sp.PASSING_DIST - delta, pos[1] + sp.CAR_WIDTH / 2 + sp.PASSING_DIST - delta)
    pt3 = Point(pos[0] + sp.CAR_LENGTH / 2 + sp.PASSING_DIST - delta, pos[1] - sp.CAR_WIDTH / 2 - sp.PASSING_DIST + delta)
    pt4 = Point(pos[0] - sp.CAR_LENGTH / 2 - sp.PASSING_DIST + delta, pos[1] - sp.CAR_WIDTH / 2 - sp.PASSING_DIST + delta)
    poly = Polygon([[p.x, p.y] for p in [pt1, pt2, pt3, pt4]])
    return affinity.rotate(poly, angle, 'center')

def plot_shapely_polygon(poly, ax):
    """
    Plots a shapely polygon to the given plot data.

    :param poly: the polygon to be plotted
    :type poly: Polygon
    :param ax: the given plot data series
    :type ax: plt.Axes
    """
    ax.plot(*poly.exterior.xy)

def plot_sumo_agent(veh_id, ax):
    """
    Plots a SUMO vehicle by it's id to the given plot data series.

    :param veh_id: The ID of the vehicle
    :type veh_id: str
    :param ax: The given plot data series
    :type ax: plt.Axes
    """
    plot_shapely_polygon(get_sumo_vehicle_as_poly(veh_id), ax)

def spawn_agent_at_position(agent, route_id="route1", color=None):
    """
    Spawns an agent to the network at the given position

    :param start_pos: The starting position of the agent
    :type start_pos: Base.Node
    :param speed: the agent's starting speed
    :type speed: float
    :param depart_edge_id: the ID of the edge to depart from
    :type depart_edge_id: str
    :param depart_lane_nbr: the ID of the lane to depart
    :type depart_lane_nbr: int
    :param route_id: the ID of the route to be travelled along
    :type route_id: str
    :param agent_id: The desired ID for the agent
    :type agent_id: str
    """
    traci.vehicle.add(vehID=agent.id, routeID=route_id, departSpeed=agent.speed[0])
    if color:
        traci.vehicle.setColor(agent.id, color)
    else:
        traci.vehicle.setColor(agent.id, agent.sumo_colour)

    angle = get_sumo_angle(agent.angle)
    traci.vehicle.moveToXY(vehID=agent.id, edgeID="P_in", lane=0, x=agent.position.x, y=agent.position.y, angle=angle,
                           keepRoute=2)
    if agent.type == "bike":
        traci.vehicle.setType(agent.id, "bike")
        traci.vehicle.setVehicleClass(agent.id, "bicycle")
        agent.length = traci.vehicle.getLength(agent.id)
        agent.width = traci.vehicle.getWidth(agent.id)
    elif agent.type == "sumo":
        traci.vehicle.setType(agent.id, "sumo_car")
    elif agent.type == "autonomous":
        traci.vehicle.setType(agent.id, "autonomous")


def spawn_random_agent(speed=13.8, route_id="route1", color=None):
    """
    Adds a random agent at the given depart lane at a random position

    :param speed: the agents speed
    :type speed: float
    :param depart_lane_nbr: number of the departing lane
    :type depart_lane_nbr: int
    :param route_id: ID of the agent's route
    :type route_id: str
    :param agent_id: ID of the agent
    :type agent_id: str
    :param color: color of the random agent
    :type color: tuple
    """
    id = str(uuid.uuid1())
    traci.vehicle.add(vehID=id, routeID=route_id, departPos="random",
                      departSpeed=speed)
    if color:
        traci.vehicle.setColor(id, color)

    return id


def recolor_agent(veh_id, color):
    """
    Changes the colour of an agent to another colour.

    :param veh_id: the ID of the vehicle
    :type veh_id: str
    :param color: the new color given as a rgba value
    :type color: tuple[float, float, float]
    """
    traci.vehicle.setColor(veh_id, color)

def recolor_agent_list(conflict_veh_list, color):
    """
    Changes the colour of all the agents in the given list.

    :param conflict_veh_list: List containing all the agents
    :type conflict_veh_list: list[str]
    :param color: the new colour given as a rgba value
    :type color: tuple[float, float, float]
    """
    all_vehicle_list = traci.vehicle.getIDList()
    for agent in all_vehicle_list:
        if agent in conflict_veh_list and agent != sp.EGO_VEHICLE_ID:
            recolor_agent(agent, color)


def plot_current_view(ego_veh_id, veh_id_list):
    """
    Plots the current vehicles into a matplotlib plot

    :param ego_veh_id: the ID of the ego vehicle
    :type ego_veh_id: str
    :param veh_id_list: List of all the other vehicles to be plotted
    :type veh_id_list: list[str]
    """
    fig = plt.figure()
    ax = plt.subplot(111)
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 3000)
    plt.axis('equal')

    # plot ego vehicle
    ego_pos = traci.vehicle.getPosition(ego_veh_id)
    ego_angle = get_plt_angle(traci.vehicle.getAngle(ego_veh_id))
    Base.plot_agent(ego_pos, ego_angle, ax, clr=color.mpl_line_red)
    
    for id in veh_id_list:
        veh_pos = traci.vehicle.getPosition(id)
        veh_angle = get_plt_angle(traci.vehicle.getAngle(id))
        Base.plot_agent(veh_pos, veh_angle, ax)

    plt.show()


class Scenario:
    def __init__(self, id, network):
        """
        Constructor of Scenario Class

        :param id: id of the current scenario
        :type id: str
        :param network: the current sumo network
        :type network: CustomSumoNetVis.Net
        """
        self.id = id
        self.sumo_net = network
        self.edge_ids = self.get_edge_ids()
        self.average_ego_trajectory = None
        self.datasets = []
        self.ego_agents = []

    def get_dataset(self, name):
        """
        Gets one dataset of the current scenario by its name

        :param name: name of the dataset
        :type name: str
        :return: returns the dataset if found
        :rtype: Dataset
        """
        for data in self.datasets:
            if data.name == name:
                return data
        return None

    def get_edge_ids(self):
        """
        This functions gets all the edge IDs for the current network

        :return: A list containing the IDs of all edges in the network
        :rtype: list[str]
        """
        id_list = []
        for edge in self.sumo_net.edges:
            if edge.function != "internal":
                id_list.append(edge.id)
        return id_list

    def plot_scenario(self, ax, use_average_trajectory=True):
        """
        Plots the current scenario into a given plot data set

        :param ax: the given plot data set
        :type ax: plt.Axes
        :param use_average_trajectory: if True, only the average trajectory of the ego vehicle will be plotted
        :type use_average_trajectory: bool
        """
        if use_average_trajectory:
            ego_list = []
            for data in self.datasets:
                ego_list.append(data.ego)
            self.average_ego_trajectory = self.get_average_trajectory(ego_list)

        for dataset in self.datasets:
            dataset.plot(ax, use_average_trajectory)

    def import_data_set(self,  data_set_name):
        """
        Imports a single dataset from the current scenario

        :param data_set_name: Name of the file
        :type data_set_name: str
        """
        current_scenario_path = os.path.join(paths.scenarios, self.id)
        print(f"Reading File: {data_set_name}...")
        current_file_path = os.path.join(current_scenario_path, data_set_name)
        self.datasets.append(Dataset(self, current_file_path))
        print("Done reading!")

    def import_scenario(self):
        """
        Imports the whole scenario with the given name

        # data looks as follow: \n
        # ________global_time  time_step   pos_x   pos_y   speed   direction \n
        # row 1 ⌈ \n
        # row 2 | \n
        # row 3 | \n
        # ..........   | \n
        # row n ⌊ \n

        :param scenario_name: name of the scenario to be imported
        :type path_to_csv: str
        """
        for files in os.walk(os.path.join(paths.scenarios, self.id)):
            current_scenario_path = os.path.join(paths.scenarios, self.id)
            for file_name in files[2]:
                print(f"Reading File: {file_name}...")
                current_file_path = os.path.join(current_scenario_path, file_name)
                self.datasets.append(Dataset(self, current_file_path))

    def get_average_trajectory(self, add_header=False):
        """
        Calculates the average trajectory for a given list of agents

        :param agent_list: list containing the agents
        :type agent_list: list[Base.Agent]
        :return: the x and y values of the average trajectory
        :rtype: tuple[list[float], list[float]]
        """
        min_evaluation_steps = inf
        critical_agent = None
        for agent in self.ego_agents:
            if agent.trajectory.shape[0] < min_evaluation_steps:
                min_evaluation_steps = agent.trajectory.shape[0]
                critical_agent = agent


        x_values = np.zeros((min_evaluation_steps))
        y_values = np.zeros((min_evaluation_steps))
        speeds = np.zeros((min_evaluation_steps))
        directions = np.zeros((min_evaluation_steps))


        trajectory = np.ndarray((min_evaluation_steps, 6))
        trajectory[:, 0] = critical_agent.trajectory[:, 0]
        trajectory[:, 1] = critical_agent.trajectory[:, 1]


        for i in range(0, min_evaluation_steps):
            x_val, y_val, dir, speed = 0, 0, [], 0
            for agent in self.ego_agents:
                x_val += agent.trajectory[i, 2]
                y_val += agent.trajectory[i, 3]
                speed += agent.trajectory[i, 4]
                if agent.trajectory[i, 5] < 0:
                    dir. append(agent.trajectory[i, 5] + 2 * pi)
                else:
                    dir.append(agent.trajectory[i, 5])
            x_values[i] = x_val / len(self.ego_agents)
            y_values[i] = y_val / len(self.ego_agents)
            speeds[i] = speed / len(self.ego_agents)
            directions[i] = angle_mean(dir)

        trajectory[:, 2] = x_values
        trajectory[:, 3] = y_values
        trajectory[:, 4] = speeds
        trajectory[:, 5] = directions

        dataset = pd.DataFrame({'time': critical_agent.trajectory[:, 0], 'sim_time': critical_agent.trajectory[:, 1],
                                'TESIS_0_x': x_values, 'TESIS_0_y': y_values, 'TESIS_0_speed': speeds,
                                'TESIS_0_angle': directions})

        return trajectory, dataset


class Dataset:
    def __init__(self, scenario, path_to_csv):
        """
        Constructor Class for one dataset of a scenario

        :param scenario: the parent scenario
        :type scenario: Scenario
        :param path_to_csv: path to the dataset's .csv file
        :type path_to_csv: str
        """
        self.path = path_to_csv
        self.name = (self.path.split("\\")[-1])[:-4]
        self.scenario = scenario
        self.ego = Base.Agent()
        self.conflict_agents = []
        self.flow_agents = []
        self.time_steps = np.array((0, 0))
        self.read_data()
        self.simulation_step = 0
        self.complete_agents = []

    def plot(self, ax, use_average_ego = False):
        """
        Plots the current dataset into a given plot data series

        :param ax: plot data series
        :type ax: plt.Axes
        :param use_average_ego: if True the average ego agent trajectory will be plotted
        :type use_average_ego: bool
        """
        if not use_average_ego:
            self.ego.plot_trajectory(ax, 'r-')
        else:
            ax.plot(self.scenario.average_ego_trajectory[0], self.scenario.average_ego_trajectory[1], 'r-')
        for agent in self.flow_agents:
            agent.plot_trajectory(ax, 'c-')
        for agent in self.conflict_agents:
            agent.plot_trajectory(ax, 'b-')


    def read_data(self):
        """
        Reads the data of a given dataset into my class structure

        :param path_to_csv: path to the dataset
        :type path_to_csv: str
        """
        self.import_ego_agent()
        self.import_conflict_agents()
        self.import_flow_agents()

    def import_ego_agent(self):
        """
        Imports the ego agent of the given dataset

        :param path_to_csv: path to the dataset
        :type path_to_csv: str
        """
        df = pd.read_csv(self.path, usecols=['time', 'sim_time', 'TESIS_0_x', 'TESIS_0_y',
                                               'TESIS_0_speed', 'TESIS_0_angle'])

        self.ego = Base.Agent(autonomous=False, sumo_data=df.values, type="bike")
        self.scenario.ego_agents.append(self.ego)
        self.time_steps = self.ego.trajectory[:, 0]

    def import_conflict_agents(self):
        """
        Imports the conflict vehicle(s) from the given dataset

        :param path_to_csv: path to the dataset
        :type path_to_csv: str
        """
        relevant_columns = ['time', 'sim_time']
        df = pd.read_csv(self.path)
        for column in df.columns:
            if "conflict" in column:
                relevant_columns.append(column)

        df_conflict = pd.read_csv(self.path, usecols=relevant_columns)
        for i in range(2, df_conflict.values.shape[1] - 5, 4):
            if isnan(df_conflict.values[0, i]):
                continue
            else:
                data_length = df_conflict.values.shape[0]
                agent_data = np.ndarray([data_length, 6])
                agent_data[:, 0] = df_conflict.values[:, 0]
                agent_data[:, 1] = df_conflict.values[:, 1]
                agent_data[:, 2] = df_conflict.values[:, i]
                agent_data[:, 3] = df_conflict.values[:, i + 1]
                agent_data[:, 4] = df_conflict.values[:, i + 2]
                agent_data[:, 5] = df_conflict.values[:, i + 3]
                self.conflict_agents.append(Base.Agent(autonomous=False, sumo_data=agent_data, type="conflict"))

    def import_flow_agents(self):
        """
        Imports the flow vehicle(s) from the given dataset

        :param path_to_csv: path to the dataset
        :type path_to_csv: str
        """
        relevant_columns = ['time', 'sim_time']
        df = pd.read_csv(self.path)
        for column in df.columns:
            if self._column_is_relevant(column):
                relevant_columns.append(column)

        df_conflict = pd.read_csv(self.path, usecols=relevant_columns)
        for i in range(2, df_conflict.values.shape[1] - 5, 4):
            if isnan(df_conflict.values[0, i]):
                continue
            else:
                data_length = df_conflict.values.shape[0]
                agent_data = np.ndarray([data_length, 6])
                agent_data[:, 0] = df_conflict.values[:, 0]
                agent_data[:, 1] = df_conflict.values[:, 1]
                agent_data[:, 2] = df_conflict.values[:, i]
                agent_data[:, 3] = df_conflict.values[:, i + 1]
                agent_data[:, 4] = df_conflict.values[:, i + 2]
                agent_data[:, 5] = df_conflict.values[:, i + 3]
                self.flow_agents.append(Base.Agent(autonomous=False, sumo_data=agent_data, type="flow"))

    def _column_is_relevant(self, column):
        for edge_id in self.scenario.edge_ids:
            if edge_id in column:
                return True
        return False

    def plot_conflicting_agents(self, ax, style='b-'):
        """
        Plots all the conflicting agents into a given plot data series

        :param ax: the plot data series
        :type ax: plt.Axes
        :param style: line style of the agent's trajectory
        :type style: str
        """
        for agent in self.conflict_agents:
            agent.plot_trajectory(ax, style=style)

    def plot_flow_agents(self, ax, style='c-'):
        """
        Plots all the flow agents into a given plot data series

        :param ax: the plot data series
        :type ax: plt.Axes
        :param style: line style of the agent's trajectory
        :type style: str
        """
        for agent in self.flow_agents:
            agent.plot_trajectory(ax, style)

    def matplot_simulate(self, speed_factor=1):
        """
        Simulates the current dataset within a matplotlib plot

        """
        fig = plt.figure()
        ax = plt.subplot(111)
        self.scenario.sumo_net.plot(ax)
        plt.axis('equal')
        plt.pause(.1)

        complete_agents = [self.ego] + self.conflict_agents + self.flow_agents
        current_rects = None
        for i in range(0, self.time_steps.shape[0] - 1, speed_factor):
            if current_rects:
                for rect in current_rects:
                    ax.patches.remove(rect)
            current_rects = []
            for agent in complete_agents:
                agent.update_trajectory(i)
                agent.angle = get_plt_angle(agent.angle)
                current_rects.append(agent.plot(ax))
            plt.pause(.0001)

        plt.show()

    def sumo_simulate(self):
        """
        Simulates the current dataset within SUMO

        """

        traci.route.add("route1", ["P_in", "gneE61"])
        complete_agents = [self.ego] + self.conflict_agents + self.flow_agents
        id = 0
        for i in range(0, self.time_steps.shape[0] - 1):
            # create agents first
            for agent in complete_agents:
                agent.update_trajectory(i)
                if i == 0:
                    agent.id = str(id)
                    spawn_agent_at_position(agent)
                    if agent.id == 0:
                        traci.vehicle.setVehicleClass(agent.id, "bicycle")
                        traci.vehicle.setType(agent.id, "bike")
                    id += 1
                else:
                    agent.sumo_move()

            traci.simulationStep()


class Route:
    def __init__(self, network, route_id, start_edge_id, end_edge_id):
        self.id = route_id
        self.sumo_net = network
        self.from_edge_id = start_edge_id
        self.to_edge_id = end_edge_id
        self.from_edge = self.sumo_net.get_edge_by_id(start_edge_id)
        self.to_edge = self.sumo_net.get_edge_by_id(end_edge_id)


    def make_sumo_route(self):
        traci.route.add(self.id, [self.from_edge_id, self.to_edge_id])



def create_scenario(start_point, end_point, start_edge_id, start_lane_nbr, end_edge_id, end_lane_nbr):
    """
    Creates a scenario including an autonomous agent

    :param start_point: the agent's starting position
    :type start_point: Base.Node
    :param end_point: the agent's desired end point
    :type end_point: Base.Node
    :param start_edge_id: the ID of the agent's starting edge
    :type start_edge_id: str
    :param start_lane_nbr: the number of the starting lane
    :type start_lane_nbr: int
    :param end_edge_id: the ID of the ending edge
    :type end_edge_id: str
    :param end_lane_nbr: the number of ending lane
    :type end_lane_nbr: int
    :return: returns the created autonomous agent
    :rtype: Base.Agent
    """
    agent_route = [start_edge_id, end_edge_id]
    start_lane_id = start_edge_id + "_" + str(start_lane_nbr)
    end_lane_id = end_edge_id + "_" + str(end_lane_nbr)

    ego_agent = Base.Agent(start_point, sp.CAR_LENGTH, sp.CAR_WIDTH, sp.MIN_CURVATURE)
    ego_agent.type = "autonomous"
    lidar = Base.Sensor(ego_agent, 100, 30, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2))
    ego_agent.add_sensor(lidar)

    ego_agent.controller = control.SensorController(ego_agent, ego_agent.position, end_pos=end_point)
    ego_agent.controller.initialize(network_settings.net, sp.CONFLICT_DIST, start_lane_id, end_lane_id)

    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    sumo_cmd = [paths.exe, "-c", paths.config_file, "--start"]

    traci.start(sumo_cmd)
    traci.route.add("agent_route", agent_route)

    ego_agent.add_to_sumo(sp.EGO_VEHICLE_ID, "agent_route")
    ego_agent.set_destination(end_point)

    return ego_agent


def angle_interpol(a1, w1, a2, w2):

    diff = a2 - a1
    if diff > 180: a1 += 360
    elif diff < -180: a1 -= 360

    aa = (w1 * a1 + w2 * a2) / (w1 + w2)

    if aa > 360: aa -= 360
    elif aa < 0: aa += 360

    return aa

def angle_mean(angle):

    if not angle: return 0

    aa = 0.0
    ww = 0.0

    for a in angle:
        aa = angle_interpol(aa, ww, a, 1)
        ww += 1

    return aa