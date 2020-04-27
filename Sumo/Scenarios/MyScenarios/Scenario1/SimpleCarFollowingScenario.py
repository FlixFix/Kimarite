import os
from math import radians
import HelperFcts as hf
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
sp.MAX_SPEED = 10
sp.START_SPEED = 10
sp.POSITIONING_EPS = .7
sp.MAX_STOPPING_TIME = 20
sp.MIN_GAP = 10
sp.SHOW_PLOT = True
sp.AVERAGE_ACCEL = 1
sp.CONSOLE_OUTPUT = False

# -------------------------------------------------------------------------------
# ------------------------------ Initialization ---------------------------------
# -------------------------------------------------------------------------------
network_settings = params.NETWORK("ValidationNetwork.net.xml")
paths = params.SUMO_PATHS()
shf.start_sumo()
start_point = Base.Node(1330.42, 2100, radians(180))
# conflict_start_point = Base.Node(1763.76, 787.68, radians(87))
end_point = Base.Node(900, 2100, radians(180))
start_lane_id = "gneE70_2"
end_lane_id = "gneE82_2"
route = ["gneE70", "gneE82"]
traci.route.add("ego_route", route)
# -------------------------------------------------------------------------------

# -------------------------------------------------------------------------------
# --------------------------------- Preparation ---------------------------------
# -------------------------------------------------------------------------------
# Create Agents
tum_agent = shf.create_agent_from_trajectory_data(os.path.join(ROOT_DIR, 'tum_data2.csv'))
tum_agent.print_trajectory_values()
# shf.add_sumo_bike(conflict_start_point, "ego_route", "J_in", 2, "sumo_cyclist")
ego_agent = Base.Agent(start_point, sp.CAR_LENGTH, sp.CAR_WIDTH, sp.MIN_CURVATURE)
ego_agent.set_destination(end_point)

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
# ego_agent.add_to_sumo(sp.EGO_VEHICLE_ID, "ego_route")
# traci.simulationStep()
# ego_agent.controller.simulator.set_ego_vehicle_types(["wiedemann_car", "krauss_car", "idm_car"])
# sumo_data, running_times = ego_agent.controller.simulator.run_simulation_without_av(ego_route=route)
# print(running_times)
# ego_agent.controller.simulator.create_results(sumo_data)

# -------------------------------------------------------------------------------


# -------------------------------------------------------------------------------
# --------------------------------- Simulation ----------------------------------
# -------------------------------------------------------------------------------
ego_agent.add_to_sumo(sp.EGO_VEHICLE_ID, "ego_route")
ego_agent.controller.simulator.add_scenario_agent(tum_agent, tum_agent.id)
traci.simulationStep()
ego_agent.controller.sensor_based_control()

print(ego_agent.controller.data_monitor.timer.times)
print(ego_agent.controller.data_monitor.timer.sim_time)
print(ego_agent.controller.data_monitor.timer.real_sim_time)
ego_agent.controller.data_monitor.timer.print_total()
print("Simulation Done!")

