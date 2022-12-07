import os
from math import radians

import traci

import Paths as params
import Sumo.EnhancedController as enhControl
import Sumo.SumoHelperFcts as shf
from PathPlanningTools import Base
from Sumo import SumoParameters as sp

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
# -------------------------------------------------------------------------------
# ------------------------- Used TUM Scenario: NONE -----------------------------
# -------------------------------------------------------------------------------

# Parameters
sp.MAX_SPEED = 18
sp.START_SPEED = 10
POSITIONING_EPS = .7
MAX_STOPPING_TIME = 3
MIN_GAP = 10
sp.JUNCTION_TIME_GAP = 6
sp.SHOW_PLOT = False


# -------------------------------------------------------------------------------
# ------------------------------ Initialization ---------------------------------
# -------------------------------------------------------------------------------
network_settings = params.NETWORK("ValidationNetwork.net.xml")
paths = params.SUMO_PATHS()
shf.start_sumo()
start_point = Base.Node(1764.22, 740.23, radians(87))
conflict_start_point = Base.Node(1810, 874, radians(180))
end_point = Base.Node(1764, 990, radians(89))
start_lane_id = "J_in_2"
end_lane_id = "gneE50_1"
route = ["J_in", "gneE50"]
crossing_route = ["gneE37", "gneE41"]
traci.route.add("ego_route", route)
traci.route.add("conflict_route", crossing_route)
# -------------------------------------------------------------------------------

# -------------------------------------------------------------------------------
# --------------------------------- Preparation ---------------------------------
# -------------------------------------------------------------------------------
# Create Agents
# shf.add_sumo_bike(conflict_start_point, "conflict_route", "gneE37", 2, "sumo_cyclist")
sumo_agent = Base.Agent(conflict_start_point)
sumo_agent.type = "sumo"
sumo_agent.id = "conflict_agent"
shf.spawn_agent_at_position(sumo_agent, "conflict_route")
ego_agent = Base.Agent(start_point, sp.CAR_LENGTH, sp.CAR_WIDTH, sp.MIN_CURVATURE)
ego_agent.set_destination(end_point)

traci.simulationStep()
# Create Sensors/Controllers
lidar = Base.Sensor(ego_agent, 100, 30, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2))
ego_agent.add_sensor(lidar)
ego_agent.controller = enhControl.ControllingUnit(ego_agent, end_pos=end_point)
ego_agent.controller.initialize(network_settings.net, start_lane_id, end_lane_id, ROOT_DIR)
ego_agent.controller.overtaking_enabled = False
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
# traci.simulationStep()
# ego_agent.controller.sensor_based_control()
#
# print(ego_agent.controller.data_monitor.timer.sim_time)
# print(ego_agent.controller.data_monitor.timer.real_sim_time)
# ego_agent.controller.data_monitor.timer.print_total()
# print("Simulation Done!")

