from math import radians, sin

SCENARIO = 1

class Colors:
    def __init__(self):
        self.critical_vehicle = (255, 0, 0)
        self.conflicting_vehicle = (255, 128, 0)
        self.default_vehicle = (255, 255, 0)
        self.ego_vehicle = (0, 0, 255)
        self.field_of_view = (0, .4, .5, .6)
        self.sensor = (1, 0, 0, .3)
        self.trajectory = TrajectoryColors()
        self.agents = AgentColors()
        self.sensors = SensorColors()
        self.route = (1, 1, 0, .3)
        self.blocked_space = (204 / 255, 0, 0, .5)
        self.path = (204 / 255, 0, 204/255)
        self.rrt_tree = ()
        self.ego_plot_data = (0, 68 / 255, 85/255)
        self.conflict_plot_data = (170/255, 0, 0)

    def get_trajectory_color(self, speed):
        if speed == 0.0:
            return self.trajectory.stopped
        elif speed > 0 and speed < 2.78:
            return self.trajectory.low
        elif 2.78 <= speed <= 8.33:
            return self.trajectory.low_to_medium
        elif 8.33 < speed <= 13.9:
            return self.trajectory.medium
        elif 13.9 < speed <= 22.22:
            return self.trajectory.medium_to_high
        else:
            return self.trajectory.high

    def get_agent_colour(self, type="default"):
        if type=="critical":
            return self.agents.critical_vehicle
        elif type=="conflict":
            return  self.agents.conflicting_vehicle
        elif type=="yield":
            return self.agents.yield_vehicle
        elif type=="recognised":
            return self.agents.recognised
        else:
            return self.agents.default_vehicle

    def get_sensor_color(self, type=None):
        if type=="mid_range_scan":
            return self.sensors.mrs
        elif type=="short_range_radar":
            return self.sensors.srr
        elif type=="multi_mode_radar":
            return self.sensors.mmr
        else:
            return self.sensors.lrr

    # def get_plot_colour(self, veh_type="av"):
    #     if veh_type == "av:":
    #         return


class AgentColors:
    def __init__(self):
        self.critical_vehicle = (255, 0, 0)
        self.conflicting_vehicle = (255, 128, 0)
        self.default_vehicle = (255, 255, 0)
        self.yield_vehicle = (255, 0, 255)
        self.recognised = (0, 128, 255)
        self.bike = (128, 255, 0)


class SensorColors:
    def __init__(self):
        self.lrr = (76 / 255, 129 / 255, 156 / 255, .5)
        self.mrs = (76/255, 129/255, 156/255, .5)
        self.srr = (128/255, 0, 128/255, .5)
        self.mmr = (170/255, 212/255, 0, .5)



class TrajectoryColors:
    def __init__(self):
        # Vehicle stopped
        self.stopped = (0, 102 / 255, 204 / 255)
        # Speedbracket v < 10 km/h == 2.78 m/s
        self.low = (0, 1, 0)
        # Speedbracket  10 <= v <= 30 km/h == 8.33 m/s
        self.low_to_medium = (.5, 1, 0)
        # Speedbracket  30 < v <= 50 km/h == 13.9 m/s
        self.medium = (1, 1, 0)
        # Speedbracket 50 < v < =80 km/h == 22.22 m/s
        self.medium_to_high = (1, .5, 0)
        # Speedbracket v > 80 km/h
        self.high = (1, 0, 0)

# Agent Parameters
PLANNING_THRESHOLD = 5
SAFETY_DIST = 5
MAX_STEERING = 65
CAR_LENGTH = 4.3
CAR_WIDTH = 1.8
MIN_CURVATURE = (CAR_LENGTH / sin(radians(MAX_STEERING))) ** -1
MIN_GAP = 10
ABSOLUTE_MIN_GAP = 1.0
PASSING_DIST = .5
EGO_VEHICLE_ID = "FlixFixAgent"
MAX_BRAKE_DECEL = -9
COMFORT_BRAKE_DECEL = -2
FREE_DECEL = 1
AVERAGE_ACCEL = 1
MAX_ACCEL = 2.9
MAX_SPEED = 18
START_SPEED = 10
CONFLICT_DIST = 100
CHANGE_OF_ACCEL_RAT = .1
OVERTAKING_ACCEL = 2
OVERTAKING_SPEED = 15

# Colors
COLOURS = Colors()

# # Control Parameters
JUNCTION_EPS = .5

# Path Generation Parameters
DEFAULT_LANE_WIDTH = 3.3
THRESHOLD = DEFAULT_LANE_WIDTH * .9
TIME_STEP = .1
MAKE_TRAJECTORY_EPS = .0001
DIST_CHANGE_EPS = .3
SPEED_CHANGE_EPS = .1
# This Parameter is used to decide whether the autonomous agent' speed is adjusted or not.
# If the change in distance per time exceeds this limit, then the autonomous agent's speed
# will be adjusted accordingly.


# Plotting
PLOT_SENSOR = False
PLOT_CONE = True
PLOT_FIELD_OF_VIEW = False
SHOW_PLOT = False
CONSOLE_OUTPUT = False
PLOT_SUMO_AGENTS = True
ANNOTATE_TRAJECTORY = False

# Conflict Agents
CONFLICT_START_SPEED = 3
# This Parameter gives the start speed of all bike agents within the current simulation.
# For Some weird reason it has to be incredible low to work
DELTA_ANGLE = radians(20) # [degrees]
# This Parameter is used to check the difference in angle between the conflict vehicle
# and the autonomous agent. If DELTA_ANGLE is exceeded, then the agent will start to
# accelerate and the conflict vehicle's status is set to a default agent
RIGHT_BEFORE_LEFT_EPS = 10 # [degrees]
# This parameter gives a buffer to detect right before left, if the junction is not a 100% 90 degrees junction
JUNCTION_TIME_GAP = 5 # [s]
# This parameter gives a time gap to be respected at junctions and for the adjustment of the autonomous vehicle's speed
JUNCTION_PASSED_EPS = .1

SPEED_EPS = .5

SCENARIO_HAS_STOPPING_AGENT = False
STOPPING_TIME_STEP = None

OVERTAKING_DECISION_DIST = 2 * MIN_GAP
JCT_OVERTAKING_BUFFER = 2 * MIN_GAP

POSITIONING_EPS = .7
MAX_STOPPING_TIME = 3

