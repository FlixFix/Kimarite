import os
import sys
from math import pi, radians

import traci

import Paths as params
import Sumo.SumoHelperFcts as shf
from PathPlanningTools import Base
from Sumo import SumoParameters as sp
import Sumo.EnhancedController as enhControl

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
# -------------------------------------------------------------------------------
# --------------------------- Used TUM Scenario: 2A -----------------------------
# -------------------------------------------------------------------------------


# Parameters
sp.AVERAGE_ACCEL = 1
sp.MAX_ACCEL = 3
sp.MAX_SPEED = 10
sp.START_SPEED = 10
sp.MIN_GAP = 5
sp.SHOW_PLOT = False

# -------------------------------------------------------------------------------
# ------------------------------ Initialization ---------------------------------
# -------------------------------------------------------------------------------
network_settings = params.NETWORK("ValidationNetwork.net.xml")
paths = params.SUMO_PATHS()
shf.start_sumo()
start_point = Base.Node(1763, 1216, radians(90))
conflict_start_point = Base.Node(1763.71, 1256.67, radians(87))
end_point = Base.Node(1760, 1664, radians(90))
start_lane_id = "gneE40_2"
end_lane_id = "gneE44_1"
route = ["gneE40", "gneE42"]
traci.route.add("ego_route", route)
# -------------------------------------------------------------------------------

# -------------------------------------------------------------------------------
# --------------------------------- Preparation ---------------------------------
# -------------------------------------------------------------------------------
# Create Agents
# tum_agent = shf.create_agent_from_trajectory_data(os.path.join(ROOT_DIR, 'tum_data.csv'))
sumo_agent = Base.Agent(conflict_start_point)
sumo_agent.type = "bike"
sumo_agent.id = "conflict_agent"
shf.spawn_agent_at_position(sumo_agent, "ego_route")
# sumo_agent = shf.add_sumo_bike(conflict_start_point, "ego_route", "gneE40", 2, "conflict_agent")
ego_agent = Base.Agent(start_point, sp.CAR_LENGTH, sp.CAR_WIDTH, sp.MIN_CURVATURE)
ego_agent.set_destination(end_point)

# Create Sensors/Controllers
lidar = Base.Sensor(ego_agent, 100, 30, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2))
ego_agent.add_sensor(lidar)
ego_agent.controller = enhControl.ControllingUnit(ego_agent, end_pos=end_point)
ego_agent.controller.initialize(network_settings.net, start_lane_id, end_lane_id, ROOT_DIR)
ego_agent.controller.overtaking_enabled = True
# -------------------------------------------------------------------------------

# -------------------------------------------------------------------------------
# ------------------------- Simulation without AV -------------------------------
# -------------------------------------------------------------------------------
ego_agent.add_to_sumo(sp.EGO_VEHICLE_ID, "ego_route")
traci.simulationStep()
ego_agent.controller.simulator.set_ego_vehicle_types(["wiedemann_car", "krauss_car", "idm_car"])
sumo_data, running_times = ego_agent.controller.simulator.run_simulation_without_av(ego_route=route)
print(running_times)
ego_agent.controller.simulator.create_results(sumo_data)

# -------------------------------------------------------------------------------


# -------------------------------------------------------------------------------
# --------------------------------- Simulation ----------------------------------
# -------------------------------------------------------------------------------
# ego_agent.add_to_sumo(sp.EGO_VEHICLE_ID, "ego_route")
# ego_agent.controller.simulator.add_sumo_agent(sumo_agent)
# # ego_agent.controller.simulator.add_scenario_agent(tum_agent, tum_agent.id)
# traci.simulationStep()
# ego_agent.controller.sensor_based_control()
#
# print(ego_agent.controller.data_monitor.timer.sim_time)
# print(ego_agent.controller.data_monitor.timer.real_sim_time)
# ego_agent.controller.data_monitor.timer.print_total()
# print("Simulation Done!")