import os
import sys
from math import pi, radians

import traci

import Paths as params
import Sumo.SumoHelperFcts as shf
from PathPlanningTools import Base
from Sumo import SumoParameters as sp
import Sumo.EnhancedController as enhControl
import matplotlib.pyplot as plt


network_settings = params.NETWORK("ValidationNetwork.net.xml")

sp.PLOT_SENSOR = True



def make_sensor_plot():
    ego_agent = Base.Agent(Base.Node(0, 0, 0), sp.CAR_LENGTH, sp.CAR_WIDTH, sp.MIN_CURVATURE)

    # List of Sensors
    long_range = Base.Sensor(ego_agent, 200/10, 18, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2), type="long_range_radar")
    mid_range_scan = Base.Sensor(ego_agent, 160/10, 60, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2), type="mid_range_scan")
    short_range_front = Base.Sensor(ego_agent, 30/10, 80, 100, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2), type="short_range_radar")
    short_range_back_right = Base.Sensor(ego_agent, 30/10, 80, 50, 120, (0, sp.CAR_WIDTH), type="short_range_radar")
    short_range_back_left = Base.Sensor(ego_agent, 30/10, 80, 50, 240, (0, 0), type="short_range_radar")
    multi_mode_back_1 = Base.Sensor(ego_agent, 80/10, 16, 50, 180, (0, sp.CAR_WIDTH / 2), type="multi_mode_radar")
    multi_mode_back_2 = Base.Sensor(ego_agent, 30/10, 80, 50, 180, (0, sp.CAR_WIDTH / 2), type="multi_mode_radar")
    snsr_lst = [long_range, mid_range_scan, short_range_front, short_range_back_left, short_range_back_right,
                multi_mode_back_1, multi_mode_back_2]

    for snsr in snsr_lst:
        ego_agent.add_sensor(snsr)


    fig = plt.figure()
    ax = fig.add_subplot(111)
    # ax.axhline(0, 0, 10, linestyle='dashed', color=(102/255, 102/255, 102/255))
    # ax.axvline(0, 0, 10, linestyle='dashed', color=(102 / 255, 102 / 255, 102 / 255))

    ego_agent.plot(ax, True)
    # ax.grid(True)
    ax.spines['left'].set_position('zero')
    ax.spines['left'].set_color((102 / 255, 102 / 255, 102 / 255))
    ax.spines['right'].set_color('none')
    ax.spines['bottom'].set_position('zero')
    ax.spines['bottom'].set_color((102 / 255, 102 / 255, 102 / 255))
    ax.spines['top'].set_color('none')

    ax.tick_params(axis='x', colors=(102 / 255, 102 / 255, 102 / 255))
    ax.tick_params(axis='y', colors=(102 / 255, 102 / 255, 102 / 255))
    ax.set_xlim(-12, 25)
    ax.set_ylim(-9, 9)

    plt.show()
    print("Plotting Done!")


def sensor_recognition_example():
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
    shf.start_sumo()
    sp.SHOW_PLOT = True
    sp.PLOT_CONE = False

    start_point = Base.Node(2346, 103, 0)
    end_point = Base.Node(2809, 1021, 170 / 180 * pi)
    start_lane_id = "A_in_1"
    end_lane_id = "B_out_0"

    aligned_route_1 = ["A_in", "B_out"]
    aligned_route_2 = ["A_in", "G_out"]
    crossing_route_1 = ["-gneE1", "B_out"]
    crossing_route_2 = ["-gneE1", "A_out"]


    # create routes
    traci.route.add("aligned_1", aligned_route_1)
    traci.route.add("aligned_2", aligned_route_2)
    traci.route.add("crossing_1", crossing_route_1)
    traci.route.add("crossing_2", crossing_route_2)

    aligned_agents_positions = [.1, .25, .4, .5, .7, .9]
    edge_length = network_settings.net.get_edge_by_id("A_in").get_length()
    for i in range(0, len(aligned_agents_positions)):
        traci.vehicle.add("agent_" + str(i), "aligned_1", departPos=aligned_agents_positions[i] * edge_length,
                          departSpeed=8.33)
        traci.vehicle.setType("agent_" + str(i), "sumo_car")

    ego_agent = Base.Agent(start_point, sp.CAR_LENGTH, sp.CAR_WIDTH, sp.MIN_CURVATURE)
    lidar = Base.Sensor(ego_agent, 100, 50, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2))
    lidar.rec_vehicles = True
    ego_agent.add_sensor(lidar)
    ego_agent.controller = enhControl.ControllingUnit(ego_agent, end_pos=end_point)
    ego_agent.controller.initialize(network_settings.net, start_lane_id, end_lane_id, ROOT_DIR)
    ego_agent.controller.overtaking_enabled = False

    ego_agent.add_to_sumo(sp.EGO_VEHICLE_ID, "aligned_1")
    ego_agent.set_destination(end_point)
    traci.simulationStep()
    ego_agent.controller.sensor_based_control()

if __name__ == '__main__':
    make_sensor_plot()
    sensor_recognition_example()