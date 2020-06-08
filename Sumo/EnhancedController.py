import copy
import os, sys
import time
from math import atan2, degrees, inf, pi, radians

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import shapely.ops
import traci
from shapely.geometry import LineString, MultiLineString, Point, Polygon

import HelperFcts as hf
import Paths as params
import Sumo.Navigation as navi
import Sumo.Plotting as splt
import Sumo.SumoHelperFcts as shf
import Sumo.SumoParameters as sp
from PathPlanningTools import Base as base
from PathPlanningTools import MyRRT as planner
import Sumo.Dashboard as dash


class Timing:
    def __init__(self):
        keys = ["<class 'Sumo.EnhancedController.Activator'>", "<class 'Sumo.EnhancedController.Interpreter'>",
                "<class 'Sumo.EnhancedController.Perceptor'>", "<class 'Sumo.EnhancedController.PathPlanner'>",
                "<class 'Sumo.EnhancedController.Controller'>", "<class 'Sumo.EnhancedController.Monitoring'>",
                "<class 'Sumo.EnhancedController.Simulation'>"]
        self.times = dict.fromkeys(keys, 0)
        self.sim_time = 0
        self.real_sim_time = 0
        self.sumo_sim_times = dict()
        self.single_steps = [[0], [0]]

    def track(self, unit, time):
        self.times[str(type(unit))] += time

    def plot_results(self):
        times_df = pd.DataFrame(self.times, index=[0])

        plt.figure()
        sns.set(style="whitegrid")

        y_values = ['Activator', 'Interpreter', 'Perceptor', 'PathPlanner', 'Controller', 'Monitoring', 'Simulation']
        x_values = times_df.values[0]
        graph = sns.barplot(x_values, y_values, palette='Spectral')

        splt.show_values_on_bars(graph, "h", 0.3)

        plt.show()
        plt.savefig("times.svg")
        print("Done!")

    def print_total(self):
        total_time = 0
        for data in self.times:
            total_time += self.times[data]
        print(total_time)

    def get_step_time(self):
        self.single_steps[0].append(self.single_steps[0][-1] + 1)
        self.single_steps[1].append(time.time())

    def plot_diff(self):
        x_values = self.single_steps[0][:-1]
        y_values = []
        for i in range(0, len(self.single_steps[1]) - 1):
            y_values.append(self.single_steps[1][i + 1] - self.single_steps[1][i])

        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.axhline(.1, 0, 100, linestyle='dashed', color=(0, 0, 0), label="Time Step within SUMO: 0.1 seconds")
        ax.plot(x_values, y_values, label="Measured Values for the Simulation Time Steps")

        for value, index in zip(y_values, x_values):
            if value > sp.TIME_STEP:
                ax.plot(index, value,'ro')
                # ax.annotate(str(round(value,4)), (index, value), label="Exceeded Time Steps")

        ax.legend(loc='best')
        plt.show()
        print("DONE")


TIME = Timing()


def timing(f):
    def wrap(*args):
        start_time = time.time()
        ret = f(*args)
        exec_time = time.time() - start_time
        if isinstance(args, tuple):
            unit = args[0]
            TIME.track(unit, exec_time)
        else:
            TIME.track(*args, exec_time)

        return ret

    return wrap




class ControllingUnit:
    def __init__(self, ego_agent, end_pos=None, network=None):
        self.agent = ego_agent
        # -----------------------
        # Currently used members
        self.sumo_net = network
        self.current_path = None
        self.agents_in_view = []
        self.agents_oncoming = []
        self.agents_aligned = []
        self.agents_to_yield = []
        self.agents_minor = []
        self.path_planner = None
        self.data_monitor = None
        self.perception = None
        self.interpreter = None
        self.show_plot = sp.SHOW_PLOT
        self.plotter = None
        self.destination = end_pos
        self.controller = None
        self.current_acceleration = sp.AVERAGE_ACCEL
        self.current_control = None
        self.controller_reacted = False
        self.activator = None
        self.overtaking_enabled = False
        self.simulator = None
        self.time_stopped = 0
        # ------------------------
        self.arrived = False
        self.show = sp.SHOW_PLOT
        self.output = sp.CONSOLE_OUTPUT

        self._helper = None

    def initialize(self, network, start_lane_id=None, end_lane_id=None, scenario_path=None):
        self.destination = self.agent.destination
        self.sumo_net = network

        # initialize the controllers
        # Path Planner
        self.path_planner = PathPlanner(self)
        self.path_planner.get_route(start_lane_id, end_lane_id)

        # Perceptor
        self.perception = Perceptor(self)

        # Interpreter
        self.interpreter = Interpreter(self)

        # Controller
        self.controller = Controller(self)

        # Activator
        self.activator = Activator(self)

        # Simulator
        self.simulator = Simulation(self)

        # Data Monitor
        self.data_monitor = Monitoring(self, root_path=scenario_path)
        self.plotter = Plotting(self)
        self._helper = Helper()

    def sensor_based_control(self):
        ax = None
        if self.show:
            fig = plt.figure()
            ax = plt.subplot(111)
            plt.axis('equal')
            #self.path_planner.satnav.highlight_path(self.path_planner.route_borders, ax)
            self.sumo_net.plot(ax, plot_markings=False)

        # Finish Control Loop, when Agent arrived at Target Region
        start_time = time.time()
        self.data_monitor.timer.single_steps[1][0] = time.time()
        while not self.arrived:

            # Do all the Updates to get a new path and trajectory
            if self.activator.ego_and_conflict_vehicle_stopped() and not self.path_planner.rrt_planning:
                if self.activator.check_for_junction_overtake():
                    continue
                else:
                    self.simulator.traci_step(self.agent.position, self.agent.angle, self.agent.get_speed()[0])
                    continue


            self.perception.find_conflicting_agents()
            if not self.path_planner.overtaking:
                _, _ = self.path_planner.border_update()
            self.current_path = self.path_planner.get_path()

            # if self.path_planner.overtaking:
            #     self.plotter.plot_overtaking_scenario(self.current_path)

            trajectory, directions, speeds = self.controller.make_trajectory()

            # Main Driving Loop
            for i in range(0, len(trajectory)):
                self.simulator.traci_step(trajectory[i], directions[i], speeds[i])


                if self.perception.check_for_arrival(5):
                    if self.show:
                        self.plotter.plot_results()
                    self.arrived = True
                    break

                # Check if end of Conflict Agent Data reached
                if self.activator.sim_agent_data_end():
                    self.arrived = True
                    break

                # Activate or Deactivate Controllers
                self.activator.conflict_agent_speed_changed()
                if self.activator.agent_finished_overtaking(trajectory):
                    break
                if self.activator.agent_passed_stopped_vehicle(trajectory):
                    break

                # Update Sensors and Conflicting Agents
                self.perception.plot_and_sensor_update(trajectory, speeds, directions, ax)
                self.perception.find_conflicting_agents()

                if self.overtaking_enabled and not self.path_planner.overtaking:
                    if self.interpreter.check_for_overtaking():
                        break

                if self.interpreter.is_enabled():
                    if self.interpreter.check_for_conflict():
                        break
                    if self.interpreter.check_for_junction_control():
                        break
                    if self.activator.acceleration_changed() and not self.path_planner.rrt_planning:
                        break

                if self.perception.junction_in_view():
                    # TODO: Activate Junction Controller
                    break


        self.data_monitor.timer.sim_time = time.time() - start_time
        self.data_monitor.make_ego_trajectory()
        self.data_monitor.timer.real_sim_time = self.agent.trajectory[-1, 0]
        # self.data_monitor.timer.plot_diff()
        # self.data_monitor.timer.plot_results()
        # self.plotter.plot_conflict_scenario()
        if self.show:
            self.plotter.plot_results()
            plt.show()
        print("Simulation Done!")

    def update_activator(self, conflict_vehicle=None, conflict_speed=None, acceleration=None):
        """
        Updates the activator and sets the current conflict variables.

        :param conflict_vehicle: the current conflict vehicle
        :type conflict_vehicle: str
        :param conflict_speed: the speed of the current conflict vehicle
        :type conflict_speed: float
        :param acceleration: the acceleration of the conflict vehicle
        :type acceleration: float
        """
        if conflict_vehicle:
            self.data_monitor.most_recent_conflict_vehicle = conflict_vehicle
        self.activator.current_conflict_vehicle = conflict_vehicle
        self.activator.previous_conflict_speed = conflict_speed
        self.activator.current_acceleration = acceleration
        if not conflict_vehicle:
            self.controller.acceleration = sp.AVERAGE_ACCEL


class Activator:
    def __init__(self, parent):
        self.control_unit = parent
        self.previous_conflict_speed = None
        self.current_conflict_vehicle = None
        self.current_acceleration = None
        self.previous_connection = None
        self.max_stopping_time = sp.MAX_STOPPING_TIME
        self.agent_to_yield = None

    @timing
    def conflict_agent_speed_changed(self):
        """
        This function checks whether the speed of the leading vehicle changed. If it changed the interpreter is
        switched on again in order to react to the new speed.

        """
        # Turn off if overtaking
        if self.control_unit.path_planner.overtaking:
            self.control_unit.interpreter.switch_state(False)
            return


        if self.current_conflict_vehicle:
            # print(traci.vehicle.getSpeed(self.current_conflict_vehicle))
            if self.previous_conflict_speed != traci.vehicle.getSpeed(self.current_conflict_vehicle):
            # if self.previous_conflict_speed != self.control_unit.simulator.get_scenario_agent_by_id(
            #         self.current_conflict_vehicle).get_speed()[0]:
                self.control_unit.interpreter.switch_state(True)
            else:
                self.control_unit.interpreter.switch_state(False)
        else:
            self.control_unit.interpreter.switch_state(True)

    @timing
    def acceleration_changed(self):
        """
        This functions checks, whether the acceleration within the controller has changed and initiates a break
        and replan if the acceleration actually changed

        :return: True if acceleration within the controller changed else False
        :rtype: bool
        """
        if self.current_acceleration != self.control_unit.controller.acceleration:
            self.current_acceleration = self.control_unit.controller.acceleration
            return True
        else:
            return False

    @timing
    def agent_finished_overtaking(self, trajectory):
        """
        This function checks if the agent finished his overtaking / reached the last point of the trajectory.

        :param trajectory: the current trajectory
        :type trajectory: list[Point]
        :return: True of the agent reached the last point else False
        :rtype: bool
        """
        if self.control_unit.path_planner.overtaking:
            if self.control_unit.controller.calc_dist_to_pos(trajectory[-1]) == 0:
                self.control_unit.controller.acceleration = sp.AVERAGE_ACCEL
                self.current_acceleration = sp.AVERAGE_ACCEL
                self.control_unit.path_planner.overtaking = False
                self.control_unit.path_planner.overtaking_length = 0
                self.control_unit.interpreter.switch_state(True)
                return True
            else:
                return False
        else:
            return False

    @timing
    def agent_passed_stopped_vehicle(self, trajectory):
        """
        Checks if the agent reached the last point of the vehicle passing trajectory.

        :param trajectory: the current trajectory
        :type trajectory: list[Point]
        :return: True if the agent reached the last point else False
        :rtype: bool
        """
        if self.control_unit.path_planner.rrt_planning:
            if self.control_unit.controller.calc_dist_to_pos(trajectory[-1]) == 0:
                self.control_unit.controller.acceleration = sp.AVERAGE_ACCEL
                self.current_acceleration = sp.AVERAGE_ACCEL
                self.control_unit.path_planner.rrt_planning = False
                self.control_unit.interpreter.switch_state(True)
                return True
            else:
                return False
        else:
            return False

    @timing
    def ego_and_conflict_vehicle_stopped(self):
        """
        Checks if the ego agent and the conflict agent stopped.

        :return: True if they both stopped else False
        :rtype: bool
        """
        if self.current_conflict_vehicle:
            if self.control_unit.agent.get_speed()[0] + traci.vehicle.getSpeed(self.current_conflict_vehicle) == 0:
                self.control_unit.time_stopped += sp.TIME_STEP
                return True
        else:
            self.control_unit.time_stopped = 0
            return False

    @timing
    def check_for_junction_overtake(self):
        """
        This function checks whether an overtaking at a junction is feasible.

        :return: True if possible else False
        :rtype: bool
        """
        if self.control_unit.time_stopped > self.max_stopping_time and \
            self.control_unit.agent.get_speed()[0] + traci.vehicle.getSpeed(self.current_conflict_vehicle) == 0:
            self.control_unit.path_planner.rrt_planning = True
            self.control_unit.controller.acceleration = sp.AVERAGE_ACCEL
            self.control_unit.interpreter.switch_state(False)
            return True
        else:
            return False

    def sim_agent_data_end(self):
        """
        Checks if the end of the simulation agent data has been reached and breaks the simulation.

        :return: True if last data point has been reached, else False
        :rtype: bool
        """
        if self.control_unit.simulator.current_data_step == self.control_unit.simulator.scenario_agents_data_length:
            return True
        else:
            return False


class Perceptor:
    def __init__(self, parent):
        self.control_unit = parent
        self.field_of_view = None
        self.max_dist = 0
        self.add_field_of_view()

    def add_field_of_view(self):
        """
        Adds the field of view to the perceptor.
        """
        fov = base.FieldOfView(self.control_unit.agent, 100, 130, 50, 0, (sp.CAR_LENGTH, sp.CAR_WIDTH / 2))
        self.field_of_view = fov
        self.max_dist = self.field_of_view.reach

    @timing
    def find_conflicting_agents(self):
        """
        Finds all the current agents in the field of view.

        """
        self.control_unit.agents_in_view = self.field_of_view.conflict_vehicles
        self.control_unit.agents_in_view = self.sort_conflict_agents_by_distance()
        self.sort_conflict_agents_by_direction()
        # ACTIVATE THIS LINE FOR VEHICLE RECOGNITION OF THE SENSOR!
        # self.control_unit.agent.conflict_vehicles = self.control_unit.agents_in_view
        # ----------------------------------------------------------
        if len(self.control_unit.agents_aligned) == 0 and not self.control_unit.path_planner.rrt_planning\
                and not self.control_unit.activator.agent_to_yield:
            self.control_unit.update_activator(acceleration=self.control_unit.activator.current_acceleration)

    def sort_conflict_agents_by_distance(self):
        """
        Sorts all the agents in the current field of view by ascending distance.

        :return: Agents in view sorted by ascending distance
        :rtype: list[str]
        """
        distances = dict()
        for i, v in enumerate(self.control_unit.agents_in_view):
            distances[i] = ((traci.vehicle.getPosition(v)[0] - self.control_unit.agent.global_position[0]) ** 2 + (
                    traci.vehicle.getPosition(v)[1] - self.control_unit.agent.global_position[1]) ** 2) ** .5

        self.control_unit.agents_in_view = [v for (d, v) in sorted(zip(distances, self.control_unit.agents_in_view),
                                                                   key=lambda p: p[0])]
        shf.recolor_agent_list(self.control_unit.agents_in_view, sp.COLOURS.conflicting_vehicle)
        return [v for (d, v) in sorted(zip(distances, self.control_unit.agents_in_view), key=lambda p: p[0])]

    def sort_conflict_agents_by_direction(self):
        """
        Sorts the recognised agents by their direction of travel.
        """
        self.control_unit.agents_aligned = []
        self.control_unit.agents_oncoming = []
        self.control_unit.agents_minor = []
        self.control_unit.agents_to_yield = []
        for agent in self.control_unit.agents_in_view:
            self.get_agent_role(agent)

        self.control_unit.simulator.recolor_agents()

    def get_agent_role(self, agent):
        """
        Sorts the agents according to their direction of travel and the relation to the ego vehicle.

        :param agent: the conflicting agent ID
        :type agent: str
        :return: None
        :rtype: None
        """
        conflict_angle = radians(shf.get_plt_angle(traci.vehicle.getAngle(agent))) - self.control_unit.agent.angle
        if conflict_angle < 0:
            conflict_angle += 2 * pi

        if sp.DELTA_ANGLE <= conflict_angle < pi - sp.DELTA_ANGLE:
            if self.control_unit.interpreter.conflict_left_of_agent(agent):
                self.control_unit.agents_minor.append(agent)
            else:
                self.control_unit.agents_to_yield.append(agent)
        elif pi - sp.DELTA_ANGLE <= conflict_angle < pi + sp.DELTA_ANGLE:
            self.control_unit.agents_oncoming.append(agent)
        elif pi + sp.DELTA_ANGLE <= conflict_angle < 2 * pi - sp.DELTA_ANGLE:
            self.control_unit.agents_minor.append(agent)
        else:
            self.control_unit.agents_aligned.append(agent)

    @timing
    def junction_in_view(self):
        """
        Checks whether a junction is in view in order to activate the junction controller.

        :return: True if a junction is inside the field of view
        :rtype: bool
        """
        for i in range(0, len(self.control_unit.path_planner.connections)):
            jct_pos = Point(self.control_unit.path_planner.connections[i].junction.pos_x,
                            self.control_unit.path_planner.connections[i].junction.pos_y)
            if jct_pos.within(self.field_of_view.cone):
                self.control_unit.path_planner.next_connection = self.control_unit.path_planner.connections[i]
                self.control_unit.activator.next_junction = self.control_unit.path_planner.connections[i]
                self.control_unit.path_planner.connections.pop(i)
                return True
            else:
                continue
        # self.control_unit.path_planner.next_connection = None
        # self.control_unit.activator.next_junction = None
        return False

    @timing
    def plot_and_sensor_update(self, trajectory, speeds, directions, ax=None):
        """
        Updates the agents's sensors and the current field of view and plots everything.

        :param trajectory: the current agent's trajectory
        :type trajectory: list[Point]
        :param speeds: the respective speed values
        :type speeds: list[float]
        :param directions: the respective directions in radians
        :type directions: list[float]
        :param ax: the given plot data series
        :type ax: plt.Axes
        """
        if not self.control_unit.path_planner.overtaking:
            self.control_unit.agent.do_sensor_update(self.control_unit.path_planner.route_borders, self.max_dist)
        self.field_of_view.update_field_of_view()
        if ax and self.control_unit.show_plot:
            self.control_unit.plotter.plot_update(trajectory, speeds, directions, ax)
            self.control_unit.plotter.plot_borders(ax)
            plt.pause(.0000001)

        # # TODO: Remove after Debug
        # if self.control_unit.path_planner.overtaking:
        #     self.control_unit.plotter.plot_update(trajectory, speeds, directions, ax)
        #     plt.pause(.0000001)

    @timing
    def check_for_arrival(self, derivation):
        """
        Checks whether the agent reached the target region, given a derivation.

        :param derivation: size of the target area
        :type derivation: float
        :return: True if the target has been reached
        :rtype: bool
        """
        dist_to_destination = ((self.control_unit.agent.position.x - self.control_unit.destination.x) ** 2 +
                               (self.control_unit.agent.position.y - self.control_unit.destination.y) ** 2) ** .5
        if dist_to_destination < derivation:
            return True
        return False


class Interpreter:
    def __init__(self, parent):
        self.control_unit = parent
        self.current_conflict_veh = None
        self._activated = True
        self.overtaking_length = 0

    def switch_state(self, enabled):
        """
        Switches the Interpreter on or off and assigns the average acceleration to the controller in order to move

        :param enabled: activates the interpreter if set to True
        :type enabled: bool
        """
        self._activated = enabled
        if self._activated:
            self.control_unit.controller.previous_conflict_speed = 0
            self.control_unit.controller.acceleration = sp.AVERAGE_ACCEL

    def is_enabled(self):
        """
        Checks whether the interpreter is currently activated or not.

        :return: True if activated else False
        :rtype: bool
        """
        return self._activated

    @timing
    def check_for_conflict(self):
        if self.check_for_aligned_conflict():
            if self.control_unit.controller.react_to_aligned_traffic(self.current_conflict_veh):
                # DEBUG
                conflict_pos = traci.vehicle.getPosition(self.current_conflict_veh)
                # print(conflict_pos)
                # print(self.control_unit.controller.calc_dist_to_pos(conflict_pos))
                # DEBUG
                return True
        self.check_for_oncoming_conflict()
        self.current_conflict_veh = None
        return False

    def check_for_aligned_conflict(self):
        if len(self.control_unit.agents_aligned) > 0:
            self.current_conflict_veh = self.control_unit.agents_aligned[0]
            # print(traci.vehicle.getSpeed(self.current_conflict_veh))
            if traci.vehicle.getSpeed(self.current_conflict_veh) > self.control_unit.agent.get_speed()[0]:
                # self.control_unit.controller.acceleration = sp.AVERAGE_ACCEL
                return False
            else:
                return True
        else:
            return False

    def check_for_oncoming_conflict(self):
        # This will only be interesting when it comes to junctions and a left turn
        # TODO: Implement this for a left turn - not in this thesis.
        print("")

    @timing
    def check_for_overtaking(self):
        """
        Checks whether overtaking is an option or not. This function also sets the overtaking acceleration and turns off
        the interpreter for the overtaking maneuver.

        :return: True if overtaking is safe else False
        :rtype: bool
        """
        if len(self.control_unit.agents_aligned) < 0: self.current_conflict_veh = self.control_unit.agents_aligned[0]
        if self.current_conflict_veh:
            conflict_with_junction = False
            conflict_with_oncoming = False

            dist_to_conflict = self.control_unit.controller.calc_dist_to_vehicle(self.current_conflict_veh)
            if dist_to_conflict > sp.OVERTAKING_DECISION_DIST:
                return False

            overtaking_time = self.calc_overtaking_time()
            overtaking_length = self.calc_overtaking_length(overtaking_time)

            # check conflict with oncoming traffic
            if len(self.control_unit.agents_oncoming) > 0:
                dist_veh = self.control_unit.controller.calc_dist_to_vehicle(self.control_unit.agents_oncoming[0])
                oncoming_veh_speed = traci.vehicle.getSpeed(self.control_unit.agents_oncoming[0])
                ego_speed = self.control_unit.agent.get_speed()[0]
                dist_diff = dist_veh + oncoming_veh_speed * overtaking_time - ego_speed * overtaking_time - \
                                 .5 * sp.OVERTAKING_ACCEL * overtaking_time ** 2
                if dist_diff < 2 * sp.MIN_GAP:
                    conflict_with_oncoming = True

            # check conflict with next junction
            if self.control_unit.activator.previous_connection:
                jct_pos_x = self.control_unit.activator.previous_connection.junction.pos_x
                jct_pos_y = self.control_unit.activator.previous_connection.junction.pos_y
                dist_jct = self.control_unit.controller.calc_dist_to_pos((jct_pos_x, jct_pos_y))
                if overtaking_length > dist_jct + sp.JCT_OVERTAKING_BUFFER:
                    conflict_with_junction = True

            if not conflict_with_junction and not conflict_with_oncoming:
                self.control_unit.path_planner.overtaking_length = overtaking_length
                self.control_unit.controller.acceleration = sp.OVERTAKING_ACCEL
                self.control_unit.path_planner.overtaking = True
                self.control_unit.interpreter.switch_state(False)
                return True
            else:
                return False
        else:
            return False

    @timing
    def check_for_junction_control(self):
        """
        Checks whether the acceleration should be adjusted due to a vehicle with right of way.

        :return: True if acceleration got adjusted else False
        :rtype: bool
        """
        if not self.control_unit.agents_aligned and self.control_unit.agents_to_yield \
                and self.control_unit.path_planner.next_connection:
            critical_agent = self.control_unit.agents_to_yield[0]
            self.calc_yielding_acceleration_adjustment(critical_agent)
            return True
        else:
            self.control_unit.interpreter.switch_state(True)
            return False

    def junction_start_behind_agent(self, junction_start_pt):
        """
        This function checks if the start point of the current junction is behind the agent.

        :param junction_start_pt: the starting point (stopping line) of the junction
        :type junction_start_pt: tuple[float, float]
        :return: True if the agent passed the stopping line of the junction
        :rtype: bool
        """
        dir_to_junction = atan2((junction_start_pt[1] - self.control_unit.agent.position.y), (junction_start_pt[0] -
                                                                               self.control_unit.agent.position.x))

        if 3 / 2 * pi > ((abs((self.control_unit.agent.angle) - dir_to_junction)) % (2 * pi)) > pi / 2:
            return True
        else:
            return False

    def agent_passed_junction(self, last_connection_pt):
        """
        This function check whether the agent reached the end of the junction / got very close to it using an eps value.

        :param last_connection_pt: the last alignment point of the connection alignment
        :type last_connection_pt: tuple[float, float]
        :return: returns True if the agent reached the end of the junction else False
        :rtype: bool
        """
        dist = ((self.control_unit.agent.position.x - last_connection_pt[0]) ** 2 +
                (self.control_unit.agent.position.y - last_connection_pt[1]) ** 2) ** .5
        if dist < sp.JUNCTION_EPS:
            return True
        else:
            return False

    def conflict_left_of_agent(self, conflict_veh):
        """
        Checks if a conflicting agent is right or left of the agent

        :param conflict_veh: the ID of the conflicting vehicle
        :type conflict_veh: str
        :return: True if the conflicting agent is left of the ego agent else False
        :rtype: bool
        """
        conflict_pos, conflict_angle = shf.transform_to_origin(conflict_veh, self.control_unit.agent)
        dir_to_conflict =  atan2((conflict_pos[1]),
                                 (conflict_pos[0]))
        
        if dir_to_conflict >= 0:
            return True
        else:
            return False

    def calc_overtaking_length(self, time):
        """
        Calculates the needed overtaking length based on the critical conflict vehicle's speed and the agent's speed.

        :return: the overtaking distance
        :rtype: float
        """
        ego_speed = self.control_unit.agent.get_speed()[0]
        acceleration = sp.OVERTAKING_ACCEL
        return ego_speed * time + .5 * acceleration * time ** 2

    def calc_overtaking_time(self):
        """
        This function calculates the time necessary for the overtaking maneuver.

        :return: Teh necessary overtaking time
        :rtype: float
        """
        conflict_speed = traci.vehicle.getSpeed(self.current_conflict_veh)
        ego_speed = self.control_unit.agent.get_speed()[0]
        acceleration = sp.OVERTAKING_ACCEL
        dist = self.control_unit.controller.calc_dist_to_vehicle(self.current_conflict_veh)


        t_1 = ((ego_speed - conflict_speed) +
               ((conflict_speed - ego_speed) ** 2 + 2 * acceleration *
                (dist + 3 * sp.MIN_GAP + self.control_unit.agent.length)) ** .5) / (-acceleration)

        t_2 = ((ego_speed - conflict_speed) -
               ((conflict_speed - ego_speed) ** 2 + 2 * acceleration *
                (dist + 3 * sp.MIN_GAP + self.control_unit.agent.length)) ** .5) / (-acceleration)

        if t_1 > 0:
            return t_1
        else:
            return t_2

    def calc_yielding_acceleration_adjustment(self, crit_veh):
        """
        Calculates the change in acceleration in case a yielding vehicle is detected.

        :param crit_veh: the vehicle with right of way
        :type crit_veh: str
        """
        self.control_unit.data_monitor.most_recent_connection = self.control_unit.path_planner.next_connection
        jctn_pos_x = self.control_unit.path_planner.next_connection.junction.pos_x
        jctn_pos_y = self.control_unit.path_planner.next_connection.junction.pos_y
        agent_dist_to_jctn = self.control_unit.controller.calc_dist_to_pos((jctn_pos_x, jctn_pos_y)) - self.control_unit.agent.length / 2
        conflict_dist_to_jctn = self.control_unit.controller.calc_dist_to_pos((jctn_pos_x, jctn_pos_y),
                                                                             origin=traci.vehicle.getPosition(crit_veh)) - traci.vehicle.getLength(crit_veh) / 2
        ego_speed = self.control_unit.agent.get_speed()[0]
        acceleration = self.control_unit.activator.current_acceleration

        t_1 =  (- ego_speed + (ego_speed ** 2 + 2 * acceleration * agent_dist_to_jctn) ** .5) / acceleration
        t_2 = (- ego_speed - (ego_speed ** 2 + 2 * acceleration * agent_dist_to_jctn) ** .5) / acceleration

        if t_1 < 0:
            agent_time_to_arrival = t_2
        else:
            agent_time_to_arrival = t_1
        conflict_speed = traci.vehicle.getSpeed(crit_veh)
        conflict_time_to_arrival = conflict_dist_to_jctn / conflict_speed

        if abs(conflict_time_to_arrival - agent_time_to_arrival) <= sp.JUNCTION_TIME_GAP:
            self.control_unit.activator.agent_to_yield = crit_veh
            if agent_time_to_arrival < conflict_time_to_arrival:
                desired_arrival = conflict_time_to_arrival - sp.JUNCTION_TIME_GAP
            else:
                desired_arrival = conflict_time_to_arrival + sp.JUNCTION_TIME_GAP
            new_acceleration = 2 * (agent_dist_to_jctn - ego_speed * desired_arrival) / desired_arrival ** 2

            # Check if the new speed at the junction exceeds the max speed, if so recalculate:
            speed_at_jctn = ego_speed + desired_arrival * new_acceleration
            if round(speed_at_jctn, 0) > sp.MAX_SPEED:
                desired_arrival = conflict_time_to_arrival + sp.JUNCTION_TIME_GAP
                new_acceleration = 2 * (agent_dist_to_jctn - ego_speed * desired_arrival) / desired_arrival ** 2


            if self.control_unit._helper.on:
                self.control_unit._helper.param1 = (copy.copy(jctn_pos_x), copy.copy(jctn_pos_y))
                self.control_unit._helper.on = False

            self.control_unit.controller.acceleration = new_acceleration
            print(new_acceleration)
            # self.control_unit.activator.current_acceleration = new_acceleration
            self.control_unit.interpreter.switch_state(False)
        else:
            self.control_unit.activator.agent_to_yield = None


class PathPlanner:
    def __init__(self, parent):
        self.control_unit = parent
        self.satnav = None
        self.start_lane_id = None
        self.goal_lane_id = None
        self.route_borders = None
        self.connection_borders = None
        self.route = None
        self.connections = []
        self.left = None
        self.right = None
        self.next_connection = None
        self.overtaking = False
        self.overtaking_length = 0
        self.rrt_planning = False

    def get_route(self, start_lane_id, end_lane_id):
        """
        Gets the current route from the start lane to the target lane using A*

        :param start_lane_id: the ID of the start lane
        :type start_lane_id: str
        :param end_lane_id: the ID of the target lane
        :type end_lane_id: str
        """
        self.satnav = navi.Navigator(self.control_unit.sumo_net)
        self.route_borders, self.route, self.connections, self.connection_borders = \
            self.satnav.get_route(start_lane_id, end_lane_id)

    @timing
    def border_update(self):
        """
        Updates the current border points

        """
        current_border_pts, current_car_pts = self.control_unit.agent.do_sensor_update(self.route_borders,
                                                                                       self.control_unit.perception.max_dist)
        self.get_bounds(current_border_pts)
        self.control_unit.data_monitor.record_sensor_data(current_border_pts)
        return current_border_pts, current_car_pts

    def get_bounds(self, sensor_data):
        """
        Gets the two borders based on the sensors collected data

        :param sensor_data: the sensor's collected data
        :type sensor_data: list[np.array]
        """

        pt1_direction, pt2_direction = None, None
        sensor_data_pts = copy.copy(sensor_data)
        closest_pt = self._get_closest_to_agent(sensor_data_pts)
        sorted_border = self._get_border(closest_pt, sensor_data_pts)

        pt1_direction = atan2((closest_pt[1] - self.control_unit.agent.position.y),
                              (closest_pt[0] - self.control_unit.agent.position.x))


        closest_pt_2 = self._get_closest_to_agent(sensor_data_pts)

        # break, if only one border is recognised:
        dist = ((closest_pt_2[1] - sorted_border[-1][1]) ** 2 + (closest_pt_2[0] - sorted_border[-1][0]) ** 2) ** .5

        if dist <= sp.THRESHOLD:
            sorted_border_2 = None

        else:
            sorted_border_2 = self._get_border(closest_pt_2, sensor_data_pts)
            pt2_direction = atan2((closest_pt_2[1] - self.control_unit.agent.position.y),
                                  (closest_pt_2[0] - self.control_unit.agent.position.x))

        # type1 = self._get_side(pt1_direction, pt2_direction)
        type1 = self._get_side_easy(closest_pt)
        if type1 == "left":
            self.left = sorted_border
            self.right = sorted_border_2
        else:
            self.left = sorted_border_2
            self.right = sorted_border

        if self.left:
            self.control_unit.data_monitor.complete_left_border.extend(self.left)
        if self.right:
            self.control_unit.data_monitor.complete_right_border.extend(self.right)

    def _get_closest_to_agent(self, sensor_data_pts):
        """
        Gets the sensor data point which is closest to the agent's current position.

        :param sensor_data_pts: the data points gathered by the sensor
        :type sensor_data_pts: list[array]
        :return: the point, which is closest to the agent's current position
        :rtype: np.ndarray
        """
        min_dist = inf
        closest_pt = None
        count = 0
        closest_el = 0
        for pt in sensor_data_pts:
            dist = ((self.control_unit.agent.position.x - pt[0]) ** 2 +
                    (self.control_unit.agent.position.y - pt[1]) ** 2) ** .5
            if dist < min_dist:
                closest_pt = np.array(pt)
                min_dist = dist
                closest_el = count
            count += 1

        # TODO: FOUND IT
        try:
            sensor_data_pts.pop(closest_el)
            return closest_pt
        except:
            print("SENSOR ERROR!")
            # if len(self.control_unit.data_monitor.rrt_scenes) > 0:
            #     self.control_unit.data_monitor.rrt_scenes[0].plot()
            sys.exit()

    def _get_border(self, closest_pt, sensor_data_pts):
        """
        Gets the interpretation of a border, based on the data collected by the sensor

        :param closest_pt: entry point, which is the point closest to agent's current position
        :type closest_pt: np.ndarray
        :param sensor_data_pts: the data points collected by the sensor
        :type sensor_data_pts: list[array]
        :return: the border as a list of sorted points
        :rtype: list[np.array]
        """
        sorted_border = [closest_pt]
        element_to_remove = 0
        new_closest = None
        while len(sensor_data_pts) > 1:
            min_dist = inf
            closest_pt = sorted_border[-1]
            for i in range(0, len(sensor_data_pts)):
                dist = ((closest_pt[0] - sensor_data_pts[i][0]) ** 2 + (
                        closest_pt[1] - sensor_data_pts[i][1]) ** 2) ** .5
                if dist < min_dist:
                    new_closest = sensor_data_pts[i]
                    min_dist = dist
                    element_to_remove = i
            if min_dist >= sp.THRESHOLD:
                break
            sorted_border.append(new_closest)
            sensor_data_pts.pop(element_to_remove)

        return sorted_border

    def _get_side_easy(self, closest_pt):
        """
        New approach to getting the right side of the border

        :param closest_pt:
        :type closest_pt:
        :return:
        :rtype:
        """
        # translate scene to origin
        translated_pt = (
        closest_pt[0] - self.control_unit.agent.position.x, closest_pt[1] - self.control_unit.agent.position.y)
        # turn according to agent angle
        translated_pt = hf.rotate([0, 0], [translated_pt[0], translated_pt[1]], -self.control_unit.agent.angle)
        # get the direction of the point
        pt_direction = atan2(translated_pt[1], translated_pt[0])
        # get side
        if pt_direction > 0:
            return "left"
        else:
            return "right"

    def _get_side(self, pt1_direction, pt2_direction):
        """
        Checks whether the point lies on the right or left side of the agent, specifying if it starts the rigt or left
        border of the road.

        :param pt1_direction: direction to the first border point in rad
        :type pt1_direction: float
        :param pt2_direction: direction to the second border point in rad
        :type pt2_direction: float
        :return: the side of the road of the first point
        :rtype: bool
        """
        if pt1_direction and pt2_direction:
            if (pt1_direction > 0 and pt2_direction < 0) or (pt1_direction < 0 and pt2_direction > 0) and (
                    self.control_unit.agent.angle > - pi / 2 and self.control_unit.agent.angle < pi / 2):
                agent_angle = self.control_unit.agent.angle + 2 * pi
                pt1_direction += 2 * pi
            elif (pt1_direction < 0 and pt2_direction < 0 and self.control_unit.agent.angle < 0):
                pt1_direction = pt1_direction if pt1_direction >= 0 else pt1_direction + 2 * pi
                agent_angle = self.control_unit.agent.angle if self.control_unit.agent.angle > 0 \
                    else self.control_unit.agent.angle + 2 * pi
            else:
                pt1_direction = pt1_direction if pt1_direction >= 0 else pt1_direction + 2 * pi
                agent_angle = self.control_unit.agent.angle

            if pt1_direction - agent_angle > 0:
                return "left"
            else:
                return "right"
        else:
            pt1_direction = pt1_direction if pt1_direction >= 0 else pt1_direction + 2 * pi
            if pt1_direction - self.control_unit.agent.angle > 0:
                return "left"
            else:
                return "right"

    def _get_longer_border(self):
        """
        Finds the border, which is longer and, therefore, results in the longer path generation.

        :return: returns the longer side of the road
        :rtype: tuple[LineString, str]
        """
        length1 = 0
        length2 = 0
        for i in range(0, len(self.left) - 1):
            length1 += np.linalg.norm(self.left[i + 1] - self.left[i])

        for i in range(0, len(self.right) - 1):
            length2 += np.linalg.norm(self.right[i + 1] - self.right[i])

        if length1 > length2:
            return self.left, "left"
        else:
            return self.right, "right"

    @timing
    def get_path(self):
        """
        Returns the driving Path based on the current recognised borders.

        :return: A Linestring representing the anticipated driving path
        :rtype: LineString
        """

        if self.rrt_planning:
            path = self.make_junction_path()
            self.control_unit.data_monitor.record_path(path)
            return path

        if self.overtaking:
            path = self.make_overtaking_path(self.overtaking_length)
            self.control_unit.data_monitor.record_path(path)
            return path

        if self.next_connection:
            last_pt = self.next_connection.via.alignment.coords[-1]
            if self.control_unit.interpreter.agent_passed_junction(last_pt):
                self.next_connection = None

        if self.next_connection:
            path = self.make_turning_path()
            self.control_unit.data_monitor.record_path(path)
            return path
        else:
            if self.right and len(self.right) > 1:
                used_border = self.right
                border_side = "right"

            elif not self.chosen_side:
                if self.left and self.right:
                    used_border, border_side = self._get_longer_border()
                elif not self.left:
                    used_border = self.right
                    border_side = "right"
                else:
                    used_border = self.left
                    border_side = "left"
                self.chosen_side = "left" if border_side == "left" else "right"
            elif self.chosen_side == "left":
                if self.left and len(self.left) > 1:
                    used_border = self.left
                    border_side = "left"
                else:
                    used_border = self.right
                    border_side = "right"
            elif self.chosen_side == "right":
                if self.right and len(self.right) > 1:
                    used_border = self.right
                    border_side = "right"
                else:
                    used_border = self.left
                    border_side = "left"

            path = self.make_path(used_border, border_side)
            self.control_unit.data_monitor.record_path(path)
            return path

    def make_path(self, border, side, lane_width=sp.DEFAULT_LANE_WIDTH):
        """
        Creates a path as linestring based on the given border, which lies in the middle of the road.

        :param border: the given border
        :type border: list[array]
        :param side: the side of the border
        :type side: str
        :param lane_width: the lane width
        :type lane_width: float
        :return: the generated path as linestring
        :rtype: LineString
        """
        path = np.ndarray((len(border), 2))
        path[0] = np.array([self.control_unit.agent.position.x, self.control_unit.agent.position.y])

        if side == 'left':
            dist = lane_width / 2
        else:
            dist = - lane_width / 2

        for i in range(0, len(border) - 1):
            mid_point = np.array([(border[i][0] + border[i + 1][0]) / 2, (border[i][1] + border[i + 1][1]) / 2])
            segment = np.array([mid_point[0] - border[i][0], mid_point[1] - border[i][1]])
            perpendicular_vec = np.cross(segment, np.array([0, 0, 1]))
            perpendicular_vec = dist / np.linalg.norm(perpendicular_vec) * perpendicular_vec
            vec = mid_point + np.array([perpendicular_vec[0], perpendicular_vec[1]])
            path[i + 1] = vec

        return LineString(path)

    def make_turning_path(self):
        """
        This function creates the turning path for the agent in case, the agent is approaching a junction. This
        only works if the path in front of the junction is straight!

        :return: the approaching and junction crossing path joined
        :rtype: LineString
        """
        junction_start_pt = self.next_connection.via.alignment.coords[0]
        if self.control_unit.interpreter.junction_start_behind_agent(junction_start_pt):
            proj = self.next_connection.via.alignment.project(Point(self.control_unit.agent.position.x,
                                                                    self.control_unit.agent.position.y))
            return self._cut(self.next_connection.via.alignment, proj)[1]
        else:
            dist_to_turn_start = ((self.control_unit.agent.position.x - self.next_connection.via.alignment.coords[0][0]) ** 2 +
                                  (self.control_unit.agent.position.y - self.next_connection.via.alignment.coords[0][1]) ** 2) ** .5
            if dist_to_turn_start < sp.JUNCTION_EPS:
                return self.next_connection.via.alignment
            else:
                approach = LineString(([self.control_unit.agent.position.x, self.control_unit.agent.position.y],
                                       [junction_start_pt[0], junction_start_pt[1]]))
                turn = self.next_connection.via.alignment
                multi_line = MultiLineString([approach, turn])
                return shapely.ops.linemerge(multi_line)

    def make_overtaking_path(self, overtaking_length, side='l'):
        """
        Creates an overtaking path consisting of two clothoids and a straight section leaving enough space to overtake
        a bicycle.

        :param overtaking_length: the length of the straight section
        :type overtaking_length: float
        :param side: the side of the overtaking maneuver in regards to the overtaken vehicle
        :type side: str
        :return: the overtaking path
        :rtype: LineString
        """
        x0, y0, theta0 = self.control_unit.agent.position.x, self.control_unit.agent.position.y, self.control_unit.agent.angle
        L = 6
        kappa0 = 0
        kappa1 = .04 if side == 'l' else -.04
        s = np.linspace(0, L, 100)
        complete_x = []
        complete_y = []

        # 1. Clothoid
        sol = hf.eval_clothoid(x0, y0, 0, kappa0, kappa1, s)
        xs, ys, thetas = sol[:, 0], sol[:, 1], sol[:, 2]
        complete_x.extend(xs)
        complete_y.extend(ys)

        # 2. Clothoid
        x_vals = []
        y_vals = []
        for i in range(len(xs) - 1, 0, -1):
            new_pt = hf.rotate([xs[-1], ys[-1]], [xs[i], ys[i]], pi)
            x_vals.append(new_pt[0])
            y_vals.append(new_pt[1])
        complete_x.extend(x_vals)
        complete_y.extend(y_vals)

        # Straight Section
        x_vals = [x_vals[-1]]
        y_vals = [y_vals[-1]]
        for i in range(1, int(overtaking_length) + 1):
            x_vals.append(x_vals[-1] + 1)
            y_vals.append(y_vals[-1])
        complete_x.extend(x_vals)
        complete_y.extend(y_vals)

        # 3. Clothoid
        sol = hf.eval_clothoid(x_vals[-1], y_vals[-1], 0, 0, -kappa1, s)
        xs, ys, thetas = sol[:, 0], sol[:, 1], sol[:, 2]
        complete_x.extend(xs)
        complete_y.extend(ys)

        # 4. Clothoid
        x_vals = []
        y_vals = []
        for i in range(len(xs) - 1, 0, -1):
            new_pt = hf.rotate([xs[-1], ys[-1]], [xs[i], ys[i]], pi)
            x_vals.append(new_pt[0])
            y_vals.append(new_pt[1])
        complete_x.extend(x_vals)
        complete_y.extend(y_vals)

        # Create Complete Path
        complete = []
        for i in range(0, len(complete_x)):
            complete.append((complete_x[i], complete_y[i]))

        # Turn path if necessary
        if theta0 != 0:
            rotated = []
            for i in range(0, len(complete)):
                rotated.append(hf.rotate(complete[0], complete[i], theta0))
            return LineString(rotated)
        else:
            return LineString(complete)

    def make_junction_path(self):
        """
        Creates the vehicle passing path on a junction.

        :return: returns the passing path consisting of two dubins paths merged into a linestring
        :rtype: LineString
        """
        # fig = plt.figure()
        # ax = plt.subplot(111)
        # x_min, y_min, x_max, y_max = self.control_unit.path_planner.next_connection.junction.shape.boundary.bounds
        # ax.set_xlim(x_min, x_max)
        # ax.set_ylim(y_min, y_max)
        # plt.axis('equal')

        rrt, end_pos = self.initialize_rrt()
        rrt.rrt_star(self.control_unit.agent.position, end_pos, 2, .25, ball_radius=1)
        cheapest_node, cost = rrt.get_shortest_path()


        # poly = matplotlib.patches.Polygon(self._get_configuration_space().exterior.coords, True,
        #                                   color='grey', zorder=0)
        # ax.add_patch(poly)
        #
        if cheapest_node == None:
            cheapest_node = rrt.rrt_tree.find_node_closest_to_goal()

        # end_pos.plot(hf.Colours().tree_nearest_nd, ax)
        # rrt.rrt_tree.plot_target_path(cheapest_node, ax)
        # rrt.rrt_tree.plot(ax, style='y-')
        # plt.pause(.1)

        cheapest_node.set_child = end_pos
        end_pos.parent = cheapest_node

        return self._from_rrt_to_dubins(end_pos, rrt)

    def _cut(self, line, distance):
        """
        This function cuts a linestring into two pieces at a certain distance.

        :param line: the Linestring to be cut
        :type line: LineString
        :param distance: the distance at which the linestring is cut
        :type distance: float
        :return: returns the two linestring pieces
        :rtype: tuple[LineString, LineString]
        """
        if distance <= 0.0 or distance >= line.length:
            return [LineString(line)]
        coords = list(line.coords)
        for i, p in enumerate(coords):
            pd = line.project(Point(p))
            if pd == distance:
                return [
                    LineString(coords[:i + 1]),
                    LineString(coords[i:])]
            if pd > distance:
                cp = line.interpolate(distance)
                return [
                    LineString(coords[:i] + [(cp.x, cp.y)]),
                    LineString([(cp.x, cp.y)] + coords[i:])]

    def _get_configuration_space(self):
        """
        Gets the configuration space at a junction to be used for the RRT* path planner.

        :return: The boundary points of the configuration space
        :rtype: Polygon
        """
        via_id = self.next_connection._via_str
        mid_line = self.connection_borders[via_id]
        full_shape = []
        for pt in mid_line.coords:
            full_shape.append(pt)

        jctn_pts = self.next_connection.junction.shape.boundary.coords
        for pt in jctn_pts:
            if self._is_right_of_line(mid_line.coords[0], mid_line.coords[1], pt):
                full_shape.append(pt)


        start_pt = mid_line.coords[0]
        final_shape = [start_pt]
        while len(full_shape) > 0:
            closest, pop_index = self._find_closest(full_shape, start_pt)
            final_shape.append(closest)
            start_pt = closest
            full_shape.pop(pop_index)

        return Polygon([[p[0], p[1]] for p in final_shape])

    def _is_right_of_line(self, pt1, pt2, pt_to_check):
        """
        Checks if a point (pt_to_check) is right of a line given by pt1 nad pt2.

        :param pt1: line point 1
        :type pt1: tuple
        :param pt2: line point 2
        :type pt2: tuple
        :param pt_to_check: point to be checked
        :type pt_to_check: tuple
        :return: True if the point is right of the line else False
        :rtype: bool
        """
        return ((pt2[0] - pt1[0]) * (pt_to_check[1] - pt1[1]) - (pt2[1] - pt1[1]) * (pt_to_check[0] - pt1[0])) < 0

    def _find_closest(self, full_shape, start_pt):
        """
        Finds the closest point to a given start point within a set of points (full_shape)

        :param full_shape: the list of points to be checked
        :type full_shape: list[tuple]
        :param start_pt: the point from which to begin
        :type start_pt: tuple
        :return: the closest point within the given point set and the index within the point set
        :rtype: tuple[tuple, int]
        """
        dist = inf
        closest_pt = None
        index_to_pop = None
        i = 0
        for pt in full_shape:
            new_dist = self.control_unit.controller.calc_dist_to_pos(pt, start_pt)
            if new_dist < dist:
                dist = new_dist
                closest_pt = pt
                index_to_pop = i
            i+= 1

        return closest_pt, index_to_pop

    def initialize_rrt(self, max_iterations=1000):
        """
        Initializes the RRT* Path Finder on Junctions.

        :param max_iterations: the maximum iterations the RRT* will be run for
        :type max_iterations: int
        :return: returns the RRT Controller and the target node for the controlling
        :rtype: tuple[planner.RRT, base.Node]
        """
        config_space = self._get_configuration_space()
        plan_alg = planner.RRT(self.control_unit.agent.position, 50, max_iterations, self.route_borders)
        plan_alg.set_configuration_space(config_space)
        plan_alg.set_obstacle_function(plan_alg.junct_obstacle_fctn)
        plan_alg.set_sample_func(plan_alg.junction_sample)
        plan_alg.full_iterations = True
        plan_alg.plot_obstacles = True
        plan_alg.set_obstacles([self.control_unit.activator.current_conflict_vehicle])
        lane_end = self.next_connection.to_lane.alignment.coords[0]
        lane_end2 = self.next_connection.to_lane.alignment.coords[1]
        dir = atan2((lane_end2[1] - lane_end[1]), (lane_end2[0] - lane_end[0]))
        return plan_alg, base.Node(lane_end[0], lane_end[1], dir)

    def _from_rrt_to_dubins(self, target_node, rrt):
        """
        Does the actual RRT* planning and creates the driving path based on the rrt* trere.

        :param target_node: the node to be reached
        :type target_node: base.Node
        :param rrt: the RRT* Controller
        :type rrt: planner.RRT
        :return: returns the finished path consisting of two dubins edges merged into a linestring
        :rtype: LineString
        """
        rrt.rrt_tree.renumber_target_path(target_node)

        # create path
        start = rrt.rrt_tree.start_node
        middle_node = rrt.rrt_tree.get_node_by_nbr(int(target_node.nbr / 4))
        angles = [degrees(start.direction), degrees(target_node.direction)]
        middle_node.direction = radians(shf.angle_mean(angles))

        path = []
        first_edge = base.DubinsEdge(start, middle_node, agent_curvature=sp.MIN_CURVATURE , dt=sp.TIME_STEP)
        second_edge = base.DubinsEdge(middle_node, target_node, agent_curvature=sp.MIN_CURVATURE, dt=sp.TIME_STEP)
        path.append(first_edge)
        path.append(second_edge)

        x_complete = []
        y_complete = []
        for i in range(0, len(path)):
            x_complete.extend(path[i].segment[0])
            y_complete.extend(path[i].segment[1])

        complete = []
        for i in range(0, len(x_complete)):
            complete.append((x_complete[i], y_complete[i]))


        # conflict_pos = traci.vehicle.getPosition(self.control_unit.activator.current_conflict_vehicle)
        conflict_angle = traci.vehicle.getAngle(self.control_unit.activator.current_conflict_vehicle)

        scene = splt.RRTScene(self.control_unit.data_monitor)
        scene.create_scene(rrt.rrt_tree, self.control_unit.simulator.stopping_pos, copy.copy(conflict_angle), LineString(complete),
                           self._get_configuration_space(), target_node, self.next_connection, self.control_unit.agent)
        self.control_unit.data_monitor.rrt_scenes.append(scene)

        return LineString(complete)


class Controller:
    def __init__(self, parent):
        self.control_unit = parent
        self.acceleration = self.control_unit.current_acceleration
        self.previous_conflict_speed = 0

    def react_to_aligned_traffic(self, conflict_veh):
        """
        Calculates the new acceleration, if a aligned conflict vehicle, which is slower than the ego agent, is detected.
        """
        ego_length = self.control_unit.agent.length
        conflict_length = traci.vehicle.getLength(conflict_veh)
        new_dist_to_conflict = self.calc_dist_to_vehicle(conflict_veh) - ego_length / 2 - conflict_length / 2
        conflict_speed = traci.vehicle.getSpeed(conflict_veh)
        ego_speed = self.control_unit.agent.get_speed()[0]

        if conflict_speed == 0:
            min_gap = sp.ABSOLUTE_MIN_GAP
        else:
            min_gap = sp.MIN_GAP

        if new_dist_to_conflict - min_gap <= 0:
            self.control_unit.agent.set_speed(conflict_speed)
            self.control_unit.activator.current_acceleration = 0
            self.acceleration = 0
        else:
            time_to_min_gap = 2 * (min_gap - new_dist_to_conflict) / (conflict_speed - ego_speed)
            deceleration = (conflict_speed - ego_speed) / time_to_min_gap
                # print(deceleration)
            self.acceleration = deceleration
            self.previous_conflict_speed = conflict_speed

            self.control_unit.update_activator(conflict_veh, self.previous_conflict_speed, self.acceleration)
        return True

    @timing
    def make_trajectory(self, dt=sp.TIME_STEP):
        """
        Creates the agent's desired trajectory based on the path and a certain step length.

        :param dt: the time interval, where the path is evaluated
        :type dt: float
        :return: returns the evaluation steps of all the trajectory points and the respective direction
        :rtype: tuple[list[Point], list[float]]
        """

        trajectory = []
        direction = []
        speeds = []

        max_speed = sp.MAX_SPEED if not self.control_unit.path_planner.overtaking else sp.OVERTAKING_SPEED

        decelerate_overtaking = False

        if self.control_unit.path_planner.overtaking:
            print("test")

        distance_travelled = self.control_unit.agent.get_speed()[0] * dt

        # Coming from overtaking
        if self.control_unit.agent.get_speed()[0] > sp.MAX_SPEED:
            speed = self.control_unit.agent.get_speed()[0] - sp.AVERAGE_ACCEL * dt
        else:
            speed = min(sp.MAX_SPEED, self.control_unit.agent.get_speed()[0] + self.acceleration * dt)

        while distance_travelled < self.control_unit.current_path.length:
            speeds.append(speed)
            trajectory.append(self.control_unit.current_path.interpolate(distance_travelled))

            if  speed > sp.MAX_SPEED and not self.control_unit.path_planner.overtaking:
                speed = speed - sp.AVERAGE_ACCEL * dt
                decelerate_overtaking = True
            else:
                speed = speed + self.acceleration * dt
                decelerate_overtaking = False

            if speed > max_speed and not decelerate_overtaking:
                speed = max_speed
            elif speed - sp.MAKE_TRAJECTORY_EPS < 0:
                speed = 0
            elif abs(speed - self.previous_conflict_speed) < sp.SPEED_EPS and self.previous_conflict_speed != 0:
                speed = self.previous_conflict_speed
                self.acceleration = 0

            distance_travelled = distance_travelled + speed * dt
            next_step = self.control_unit.current_path.interpolate(distance_travelled)
            yaw = np.arctan2((next_step.y - trajectory[-1].y), (next_step.x - trajectory[-1].x))
            if speed == 0:
                direction.append(direction[-1])
                speeds.append(speed)
                trajectory.append(self.control_unit.current_path.interpolate(distance_travelled))
                direction.append(direction[-1])
                break
            direction.append(yaw)

        if len(trajectory) == 0:
            trajectory.append(self.control_unit.current_path.interpolate(distance_travelled))
            speeds.append(speed)
            direction.append(self.control_unit.agent.angle)

        return trajectory, direction, speeds

    def calc_dist_to_vehicle(self, veh):
        """
        Calculates the agent's distance to another vehicle.

        :param veh: The conflicting vehicle
        :type veh: str
        :return: the distance to the conflicting vehicle
        :rtype: float
        """
        veh_pos = traci.vehicle.getPosition(veh)
        return ((self.control_unit.agent.position.x - veh_pos[0]) ** 2 +
                (self.control_unit.agent.position.y - veh_pos[1]) ** 2) ** .5

    def calc_dist_to_pos(self, pos, origin=None):
        """
        Calculates the agents distance to another position, if origin is given the position from origin to pos is
        calculated.

        :param pos: the target position
        :type pos: tuple
        :param origin: the starting position, if None Agent.position is used
        :type origin: tuple
        :return: distance from origin to target
        :rtype: float
        """
        if origin:
           origin_pos = origin
        else:
            origin_pos = [self.control_unit.agent.position.x, self.control_unit.agent.position.y]


        if isinstance(pos, Point):
            return ((origin_pos[0] - pos.x) ** 2 +
                    (origin_pos[1] - pos.y) ** 2) ** .5
        elif isinstance(pos, tuple):
            return ((origin_pos[0] - pos[0]) ** 2 +
                    (origin_pos[1] - pos[1]) ** 2) ** .5


class Monitoring:
    def __init__(self, parent, root_path=None):
        self.control_unit = parent
        self.complete_left_border = []
        self.complete_right_border = []
        self.complete_trajectory = []
        self.complete_directions = []
        self.complete_speeds = []
        self.complete_sensor_pts = []
        self.complete_path = []
        self.rrt_scenes = []
        self.most_recent_conflict_vehicle = None
        self.most_recent_connection = None
        self.scenario_path = root_path
        self.timer = TIME
        self.dashboard = dash.Dashboard()

    def console_output(self):
        """
        Writes the current Agent data to the console.
        """
        print("Current Speed: " + str(self.control_unit.agent.speed[0]))
        print("Current Angle: " + str(self.control_unit.agent.angle))

    @timing
    def record(self, pos, dir, speed):
        """
        Records the complete trajectory of the ego agent.

        :param pos: The new position of the agent
        :type pos: Base.Node
        :param dir: the new direction of the agent
        :type dir: float
        :param speed: the new speed of the agent
        :type speed: float
        """
        self.complete_trajectory.append(copy.copy(pos))
        self.complete_directions.append(dir)
        self.complete_speeds.append(speed)
        
        self.dashboard.update_dash(self.control_unit.agent, self.control_unit.activator)

    @timing
    def record_sensor_data(self, sensor_data):
        """
        This function keeps track of all the data points gathered by the sensor.

        :param sensor_data: the newly gathered sensor pts
        :type sensor_data: list[np.array]
        """
        self.complete_sensor_pts.append(sensor_data)

    @timing
    def record_path(self, path):
        """
        Records all the planned path elements.

        :param path: the planned path element
        :type path: LineString
        """
        self.complete_path.append(path)

    def make_ego_trajectory(self):
        self.control_unit.agent.trajectory = np.ndarray((len(self.complete_speeds), 6))
        current_step = 0
        for i in range(0, len(self.control_unit.data_monitor.complete_speeds)):
            self.control_unit.agent.trajectory[i, 0] = current_step
            self.control_unit.agent.trajectory[i, 1] = current_step
            self.control_unit.agent.trajectory[i, 2] = self.complete_trajectory[i].x
            self.control_unit.agent.trajectory[i, 3] = self.complete_trajectory[i].y
            self.control_unit.agent.trajectory[i, 4] = self.complete_speeds[i]
            self.control_unit.agent.trajectory[i, 5] = self.complete_directions[i]

            current_step += sp.TIME_STEP

        self.control_unit.agent.get_distance_travelled()

    def get_distances(self, conflict_agent):
        """
        Calculates the Ego Agent'S distance to the current conflict vehicle based on their trajectories.

        :param conflict_agent: the conflicting agent to be evaluated
        :type conflict_agent: Base.Agent
        :return: the distances based on the trajectory
        :rtype: np.array
        """
        distances = np.zeros(self.control_unit.simulator.scenario_agents_data_length)
        for i in range(0, self.control_unit.simulator.scenario_agents_data_length):
            distances[i] = self.control_unit.controller.calc_dist_to_pos((conflict_agent.trajectory[i, 2],
                                                                          conflict_agent.trajectory[i, 3]),
                                                                         (self.control_unit.agent.trajectory[i, 2],
                                                                          self.control_unit.agent.trajectory[i, 3])) - \
                           self.control_unit.agent.length / - conflict_agent.length / 2
        return distances

    def get_time_gap(self, conflict_agent):
        """
        Calculates the time-gap of the arrival times of the conflicting agent and the ego agent at the next junction.

        :param conflict_agent: the current conflicting agent
        :type conflict_agent: Base.Agent
        :return: the time values and the according time gaps
        :rtype: tuple[list, list]
        """
        jctn_pos_x = self.control_unit.data_monitor.most_recent_connection.junction.pos_x
        jctn_pos_y = self.control_unit.data_monitor.most_recent_connection.junction.pos_y
        times = []
        time_gaps =  []
        current_time = 0
        for i in range(0, len(self.control_unit.agent.trajectory)):
            current_agent_pos = (self.control_unit.agent.trajectory[i, 2], self.control_unit.agent.trajectory[i, 3])

            dist = ((current_agent_pos[0] - jctn_pos_x) ** 2 +
                    (current_agent_pos[1] - jctn_pos_y) ** 2) ** .5
            if dist < sp.JUNCTION_EPS:
                break

            ego_speed = self.control_unit.agent.trajectory[i, 4]
            acceleration = self.control_unit.agent.get_acceleration()[i]
            current_conflict_pos = (conflict_agent.trajectory[i, 2], conflict_agent.trajectory[i, 3])
            agent_dist_to_jctn = self.control_unit.controller.calc_dist_to_pos((jctn_pos_x, jctn_pos_y), current_agent_pos) - self.control_unit.agent.length / 2
            conflict_dist_to_jctn = self.control_unit.controller.calc_dist_to_pos((jctn_pos_x, jctn_pos_y), current_conflict_pos) - conflict_agent.length / 2

            conflict_time_to_arrival = conflict_dist_to_jctn / conflict_agent.trajectory[i, 4]
            t_1 = (- ego_speed + (ego_speed ** 2 + 2 * acceleration * agent_dist_to_jctn) ** .5) / acceleration
            t_2 = (- ego_speed - (ego_speed ** 2 + 2 * acceleration * agent_dist_to_jctn) ** .5) / acceleration

            if t_1 < 0:
                agent_time_to_arrival = t_2
            else:
                agent_time_to_arrival = t_1

            time_gap = conflict_time_to_arrival - agent_time_to_arrival
            time_gaps.append(time_gap)
            times.append(current_time)
            current_time += sp.TIME_STEP

        return times, time_gaps

    def safe_trajectory(self, agent, file_name):
        """
        Safes the trajectory of the given agent to a .csv file

        :param agent: the given agent
        :type agent: Base.Agent
        :param file_name: the file-name for the safed .csv file
        :type file_name: str
        """
        agent.safe_trajectory_to_csv(os.path.join(self.control_unit.data_monitor.scenario_path,
                                                  (file_name + '.csv')))


class Plotting:
    def __init__(self, parent):
        self.control_unit = parent
        self.data = parent.data_monitor
        self.plotted_ego = []
        self.plotted_conflict = []

    def plot_update(self, trajectory, speeds, directions, ax, include_sumo_agents=sp.PLOT_SUMO_AGENTS, fancy=True):
        """
        Performs a plot update to the real-time plot.

        :param trajectory: the current trajectory
        :type trajectory: list[Point]
        :param speeds: the respective speeds
        :type speeds: list[float]
        :param directions: the respective directions
        :type directions: list[float]
        :param ax: the given plot data series
        :type ax: plt.Axes
        :param include_sumo_agents: If True the SUMO Agents will also be plotted else not
        :type include_sumo_agents: bool
        :param fancy: if True the fancy plotting will be used else normal plotting
        :type fancy: bool
        """
        if self.control_unit.show_plot:
            self._delete_old(ax)
            self.plot_trajectory(trajectory, speeds, directions, ax, fancy=fancy)
            ego = self.control_unit.agent.plot(ax, sp.PLOT_SENSOR, plot_cone=sp.PLOT_CONE, include_vision_field=sp.PLOT_FIELD_OF_VIEW,
                                         fancy=fancy)
            self.plotted_ego.append(ego)

            if include_sumo_agents:
                self.plot_sumo_agents(ax)
                # for agent in self.control_unit.agents_in_view:
                #     splt.plot_cyclist(agent, ax, "conflict")
            # if include_sumo_agents:
            #     splt.plot_sumo_agents(self.agent.id, self.conflict_contol.conflict_vehicle,
            #                          self.agent.conflict_vehicles, ax, True)
            plt.pause(.0000001)

    def plot_sumo_agents(self, ax):
        for agent in self.control_unit.agents_aligned:
            img = splt.plot_cyclist(agent, ax, "critical")
            self.plotted_conflict.append(img)

        for agent in self.control_unit.agents_oncoming:
            img = splt.plot_cyclist(agent, ax, "conflict")
            self.plotted_conflict.append(img)

        for agent in self.control_unit.agents_minor:
            img = splt.plot_cyclist(agent, ax, "minor")
            self.plotted_conflict.append(img)

        for agent in self.control_unit.agents_to_yield:
            img = splt.plot_cyclist(agent, ax, "yield")
            self.plotted_conflict.append(img)


    def _delete_old(self, ax):
        while len(ax.images) > 0:
            ax.images.remove(ax.images[-1])
        # while len(self.plotted_conflict) > 0:
        #     ax.images.remove(self.plotted_conflict[-1])
        #     del self.plotted_conflict[-1]
        #
        # while len(self.plotted_ego) > 0:
        #     ax.images.remove((self.plotted_ego[-1]))
        #     del self.plotted_ego[-1]


    def plot_trajectory(self, trajectory, speeds, directions, cx, style='o-', fancy=True, annotate=False):
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
                splt.plot_marker(trajectory[i], directions[i], speeds[i], cx)
        else:
            for i in range(0, len(trajectory) - 1):
                current_pt = trajectory[i]
                current_speed = speeds[i]
                next_pt = trajectory[i + 1]
                current_section_x = [current_pt.x, next_pt.x]
                current_section_y = [current_pt.y, next_pt.y]
                cx.plot(current_section_x, current_section_y, style,
                        color=sp.Colors().get_trajectory_color(current_speed))

        if annotate:
            for i in range(0, len(trajectory)):
                cx.annotate(str(round(speeds[i],3)), (trajectory[i].x + .7, trajectory[i].y), color=(1, 1, 1))

    def plot_borders(self, ax):
        """
        Plots the current borders, which have been found.

        :param ax: the data series, where to plot the borders
        :type ax: plt.Axes
        """
        if self.control_unit.path_planner.left:
            self._plot_border(self.control_unit.path_planner.left, "left", ax)
        if self.control_unit.path_planner.right:
            self._plot_border(self.control_unit.path_planner.right, "right", ax)

    def _plot_border(self, border, border_type, ax):
        """
        Plots a single border into the given plot data series.

        :param border: the border to be plotted
        :type border: list[array]
        :param border_type: left or right border
        :type border_type: str
        :param ax: the plot data series
        :type ax: plt.Axes
        """
        x_values = []
        y_values = []
        if border_type == 'right':
            style = 'g-'
        else:
            style = 'r-'
        for i in range(0, len(border)):
            x_values.append(border[i][0])
            y_values.append(border[i][1])
        # ax.plot(x_values, y_values, style, linewidth=4, linestyle=(0, (5, 1)), zorder=10)
        ax.plot(x_values, y_values, style, linewidth=4, zorder=10)

    def plot_path(self, ax, path=None, color=sp.Colors().path):
        """
        Plots the Path/ a Linestring to the given plot data series.

        :param path: the path to be plotted
        :type path: LineString
        :param ax: the given plot data series
        :type ax: plt.Axes
        """
        if not path:
            x, y = zip(*self.control_unit.current_path.coords)
        else:
            x, y = zip(*path.coords)
        ax.plot(x, y, color=color, linewidth=2, zorder=8)

    def plot_overtaking_scenario(self, overtaking_path):
        fig = plt.figure()
        ax = plt.subplot(111)
        self.control_unit.sumo_net.plot(ax, plot_markings=True)
        self.plot_path(ax, path=overtaking_path)
        # self.control_unit.agent.plot(ax, sp.PLOT_SENSOR, plot_cone=sp.PLOT_CONE,
        #                              include_vision_field=sp.PLOT_FIELD_OF_VIEW,
        #                              fancy=True)
        splt.plot_configuration_rect("conflict_agent", ax)
        self.plot_sumo_agents(ax)
        plt.pause(.001)

    def plot_sensor_data(self, ax, style='rx'):
        """
        Plots all the data gathered by the sensor to plot.

        :param ax: the given plot data series
        :type ax: plt.Axes
        :param style: the plot style of the data points
        :type style: str
        """
        for list in self.data.complete_sensor_pts:
            for pt in list:
                ax.plot(pt[0], pt[1], style)

    def plot_results(self):
        """
        Plots all the results of the current simulation.

        """
        fig = plt.figure()
        ax = plt.subplot(221)
        bx = plt.subplot(222)
        cx = plt.subplot(223)
        dx = plt.subplot(224)
        # plot itinerary
        self.control_unit.sumo_net.plot(ax, plot_markings=False)
        self.control_unit.path_planner.satnav.highlight_path(self.control_unit.path_planner.route_borders, ax)
        plt.axis('equal')

        # plot trajectory
        self.control_unit.sumo_net.plot(bx, plot_markings=False, alpha=.5)
        self.plot_trajectory(self.data.complete_trajectory, self.data.complete_speeds, self.data.complete_directions, bx)
        self._set_extents(bx)
        # plt.axis('equal')

        # plot sensor data
        self.control_unit.sumo_net.plot(cx, plot_markings=False, alpha=.5)
        self.plot_sensor_data(cx)
        self._set_extents(cx)
        # plt.axis('equal')

        # plot border data and path
        self.control_unit.sumo_net.plot(cx, plot_markings=False, alpha=.5)
        self._plot_border(self.data.complete_left_border, 'left', dx)
        self._plot_border(self.data.complete_right_border, 'right', dx)
        for path in self.data.complete_path:
            self.plot_path(dx, path)
        self._set_extents(dx)

        for scene in self.control_unit.data_monitor.rrt_scenes:
            scene.plot()

        # fig.savefig(params.SUMO_PATHS().images + "/Scenario1_Results.png", dpi=500)
        # plt.axis('equal')

    def _set_extents(self, ax):
        """
        Sets the extents of the plot to the extents of the network.

        :param ax: the given plot data series
        :type ax: plt.Axes
        """
        x_min, y_min, x_max, y_max = self.control_unit.sumo_net._get_extents()
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

    def plot_conflict_scenario(self):
        fig = plt.figure()
        ax = fig.add_subplot(221)
        bx = fig.add_subplot(222)
        cx = fig.add_subplot(223)
        dx = fig.add_subplot(224)

        # Get the conflict agent:
        if not self.control_unit.overtaking_enabled:
            conflict_agent = self.control_unit.simulator.get_scenario_agent_by_id(
                self.control_unit.data_monitor.most_recent_conflict_vehicle)

            # print(conflict_agent.trajectory.shape)
            # print(self.control_unit.agent.trajectory.shape)

            # Get the SUMO agent:
        if self.control_unit.simulator.additional_sumo_agent:
            sumo_agent = self.control_unit.simulator.additional_sumo_agent
            sumo_agent._change_recorded_trajectory_to_ndarray()
            print(sumo_agent.get_acceleration().max())
            print(sumo_agent.get_acceleration().min())
            print(sumo_agent.recorded_trajectory[:,4].max())
            sumo_agent.safe_trajectory_to_csv(os.path.join(self.control_unit.data_monitor.scenario_path,
                                                                        'sumo_agent_data.csv'))

        if self.control_unit.overtaking_enabled or not conflict_agent:
            conflict_agent = sumo_agent



        conflict_agent._change_recorded_trajectory_to_ndarray()
        print(conflict_agent.get_acceleration().min())
        print(conflict_agent.get_acceleration().max())
        self.control_unit.simulator.scenario_agents_data_length = conflict_agent.trajectory.shape[0]

        # safe trajectories
        self.control_unit.agent.safe_trajectory_to_csv(os.path.join(self.control_unit.data_monitor.scenario_path,
                                                                    'av_agent_data.csv'))
        conflict_agent.safe_trajectory_to_csv(os.path.join(self.control_unit.data_monitor.scenario_path,
                                                                    'conflict_agent_data.csv'))

        # Plot the actual trajectory
        self.control_unit.agent.get_distance_travelled()
        conflict_agent.get_distance_travelled()

        # x_new, y_new = self._smoothen(copy.copy(self.control_unit.agent.trajectory[:, 0]), copy.copy(self.control_unit.agent.distance_travelled))
        #
        # ax.plot(x_new, y_new)
        # plt.axis('equal')
        # plt.pause(.1)
        self.control_unit.agent.plot_dist_time(ax, color=sp.Colors().ego_plot_data)
        conflict_agent.plot_dist_time(ax, color=sp.Colors().conflict_plot_data)

        dist1 = self.control_unit.controller.calc_dist_to_pos((1764.22, 740.23), origin=self.control_unit._helper.param1)
        dist2 = self.control_unit.controller.calc_dist_to_pos((1810, 874), origin=self.control_unit._helper.param1)
        ax.axhline(dist1, 0, 10, linestyle='dashed')
        ax.axhline(dist2, 0, 10, linestyle='dashed')

        plt.axis('equal')
        plt.xlim(0, self.control_unit.agent.trajectory[-1, 0])
        ax.title.set_text("Trajectory")
        plt.pause(.1)

        # Plot the distance
        distances = self.control_unit.data_monitor.get_distances(conflict_agent)
        bx.plot(conflict_agent.trajectory[:, 0], distances , color=sp.Colors().ego_plot_data)
        plt.axis('equal')
        plt.xlim(0, self.control_unit.agent.trajectory[-1, 0])
        bx.title.set_text("Distance Autonomous - Conflict")
        plt.pause(.1)

        # Plot the speeds
        cx.plot(conflict_agent.trajectory[:, 0], conflict_agent.trajectory[:, 4], color=sp.Colors().conflict_plot_data)
        cx.plot(self.control_unit.agent.trajectory[:, 0], self.control_unit.agent.trajectory[:, 4], color=sp.Colors().ego_plot_data)
        plt.axis('equal')
        plt.xlim(0, self.control_unit.agent.trajectory[-1, 0])
        cx.title.set_text("Speed")
        plt.pause(.1)

        # Plot the acceleration
        # print("Ego Max Acceleration: " + str(self.control_unit.agent.get_acceleration().max()))
        # print("Ego Max Braking: " + str(self.control_unit.agent.get_acceleration().min()))
        # print("")
        # print(conflict_agent.get_acceleration().min())
        # print(conflict_agent.get_acceleration().max())
        dx.plot(conflict_agent.trajectory[:, 0], conflict_agent.get_acceleration(), color=sp.Colors().conflict_plot_data)
        dx.plot(self.control_unit.agent.trajectory[:, 0], self.control_unit.agent.get_acceleration(),
                color=sp.Colors().ego_plot_data)
        # plt.axis('equal')

        # plt.ylim(min(conflict_agent.get_acceleration().min(), self.control_unit.agent.get_acceleration().min()),
        #          max(conflict_agent.get_acceleration().max(), self.control_unit.agent.get_acceleration().max())
        dx.title.set_text("Acceleration")
        plt.xlim(0, self.control_unit.agent.trajectory[-1, 0])
        plt.pause(.1)


        # self.plot_gap_against_acceleration(conflict_agent)


    def plot_gap_against_acceleration(self, conflict_agent):
        """
        Plots the timegaps to the next junction of the ego agent and the conflicting agent to a new figure.

        :param conflict_agent: the conflicting agent to be evaluated
        :type conflict_agent: Base.Agent
        """
        fig = plt.figure()
        bx = fig.add_subplot(211)
        dx = fig.add_subplot(212)

        # Plot Time Gap:
        times, gaps = self.control_unit.data_monitor.get_time_gap(conflict_agent)
        bx.plot(times, gaps, color=sp.Colors().ego_plot_data)

        bx.title.set_text("Time Gap Autonomous - Conflict")
        bx.set_xlim(0, self.control_unit.agent.trajectory[-1, 0])
        plt.pause(.1)

        # Plot the acceleration
        dx.plot(conflict_agent.trajectory[:, 0], conflict_agent.get_acceleration(),
                color=sp.Colors().conflict_plot_data)
        dx.plot(self.control_unit.agent.trajectory[:, 0], self.control_unit.agent.get_acceleration(),
                color=sp.Colors().ego_plot_data)

        dx.title.set_text("Acceleration")
        dx.set_xlim(0, self.control_unit.agent.trajectory[-1, 0])
        plt.pause(.1)


class Simulation:
    def __init__(self, parent):
        self.control_unit = parent
        self.scenario_agents = []
        self.scenario = None
        self.stopping_lane = None
        self.stopping_time = inf
        self.stopping_pos = None
        self.timer = 0
        self.has_stopping_agent = False
        self.scenario_agents_data_length = inf
        self.current_data_step = 0
        self.additional_sumo_agent = None
        self.ego_vehicle_types = None
        self._initial_stopping_pos = None

    @timing
    def traci_step(self, new_pos, new_dir, new_speed=None):
        """
        Performs one simulation step for all vehicles within the simulation.

        :param new_pos: the new position of the ego agent
        :type new_pos: base.Node
        :param new_dir: the new direction of the ego agent
        :type new_dir: float
        :param new_speed: the new speed of the ego agent
        :type new_speed: float
        :param ax: the given plot data series
        :type ax: plt.Axes
        """
        self.control_unit.agent.set_speed(new_speed)
        self.control_unit.agent.update(new_pos.x, new_pos.y, new_dir, speed=new_speed)
        self.control_unit.agent.sumo_move(edge="-gneE41")

        # create a stopping agent
        if self.stopping_lane or self.stopping_pos:
            self.make_conflict_agent_stop()

        if self.scenario:
            self.scenario_traci_step()

        self.single_scenario_traci_step()

        traci.simulationStep()
        self.control_unit.data_monitor.record(self.control_unit.agent.position, self.control_unit.agent.angle, self.control_unit.agent.get_speed()[0])

        if self.control_unit.output:
            self.control_unit.data_monitor.console_output()

        self.control_unit.data_monitor.timer.get_step_time()

    def single_scenario_traci_step(self, set_speed_mode=True):
        """
        Creates one step within the scenario in even without a scenario attached and just some agents.
        """
        for agent in self.scenario_agents:
            # print(agent.get_speed()[0])
            # print(traci.vehicle.getSpeed(agent.id))
            if not agent.has_stopped:
                agent.update_trajectory(agent.trajectory_step)
                agent.record_trajectory()
                agent.sumo_move()
                if set_speed_mode:
                    traci.vehicle.setSpeedMode(agent.id, 0)
                traci.vehicle.setSpeed(agent.id, agent.trajectory[agent.trajectory_step, 4])
                agent.trajectory_step += 1
                self.current_data_step = agent.trajectory_step
            else:
                agent.record_trajectory()

        if self.additional_sumo_agent:
            self.additional_sumo_agent.update_trajectory()
            self.additional_sumo_agent.record_trajectory()

    def scenario_traci_step(self):
        """
        Performs a simulation step for the complete scenario.

        """
        for agent in self.control_unit.scenario.complete_agents:
            agent.update_trajectory(self.scenario.simulation_step)
            agent.sumo_move()

        if not sp.SCENARIO_HAS_STOPPING_AGENT or not sp.STOPPING_TIME_STEP == self.scenario.simulation_step:
            self.scenario.simulation_step += 1

    def add_scenario_agent(self, agent, id):
        """
        Adds a single scenario agent to the current simulation.

        :param agent: the agent to be added containing all the trajectory data
        :type agent: base.Agent
        :param id: the desired agent ID
        :type id: str
        """
        if not "route1" in traci.route.getIDList():
            traci.route.add("route1", ["P_in", "gneE61"])
        agent.id = str(id)
        shf.spawn_agent_at_position(agent)
        self.scenario_agents.append(agent)
        self._get_lowest_data_length(agent)

    def add_sumo_agent(self, agent):
        self.additional_sumo_agent = agent

    def add_scenario(self, scenario):
        """
        Adds a complete DataSet to the current simulation.

        :param scenario: the scenario to be added
        :type scenario: shf.DataSet
        """
        self.scenario = scenario
        traci.route.add("route1", ["P_in", "gneE61"])
        self.scenario.complete_agents = [scenario.ego] + scenario.conflict_agents + scenario.flow_agents
        id = 0
        for agent in self.scenario.complete_agents:
            agent.update_trajectory(scenario.simulation_step)
            agent.id = str(id)
            shf.spawn_agent_at_position(agent)
            id += 1
        scenario.simulation_step += 1

    def set_stopping_scenario(self, stopping_time, stopping_point=None, stopping_lane=None):
        if stopping_point:
            self.stopping_lane = None
            self.stopping_pos = stopping_point
            self._initial_stopping_pos = copy.copy(self.stopping_pos)

        if stopping_lane:
            self.stopping_pos = None
            self.stopping_lane = stopping_lane

        self.stopping_time = stopping_time
        self.has_stopping_agent = True

    def make_conflict_agent_stop(self):
        """
        This function stops the conflicting agent at a given location with a certain deviation or at the end of the
        given lane. If a pos is given, the agent will always stop at the position and ignore the lane.
        """
        conflict_veh = self.control_unit.activator.current_conflict_vehicle
        if conflict_veh:
            # Stopping at a certain position
            if self.stopping_pos:
                veh_pos = traci.vehicle.getPosition(conflict_veh)
                dist_to_stopping = self.control_unit.controller.calc_dist_to_pos(self.stopping_pos, veh_pos)
                if dist_to_stopping < sp.POSITIONING_EPS:
                    if self.timer <= self.stopping_time:
                        self.stopping_pos = veh_pos
                        self.get_scenario_agent_by_id(conflict_veh).has_stopped = True
                        self.get_scenario_agent_by_id(conflict_veh).set_speed(0)
                        traci.vehicle.setSpeed(conflict_veh, 0)
                        self.timer += sp.TIME_STEP
                    else:
                        traci.vehicle.setSpeed(conflict_veh, -1)
                        traci.simulationStep()
                        new_acc = traci.vehicle.getAcceleration(conflict_veh)
                        # self.control_unit.controller.acceleration = new_acc
                        self.control_unit.controller.acceleration = sp.AVERAGE_ACCEL
                        # self.control_unit.activator.current_acceleration = new_acc
                        self.timer = 0
                        self.stopping_pos = None
                        self.get_scenario_agent_by_id(conflict_veh).has_stopped = False

            # Stopping at end of a lane
            else:
                conflict_lane_pos = traci.vehicle.getLanePosition(conflict_veh)
                lane = traci.vehicle.getLaneID(conflict_veh)
                if lane == self.stopping_lane:
                    lane_length = traci.lane.getLength(lane)
                    if abs(conflict_lane_pos - lane_length) < sp.JUNCTION_EPS:
                        if self.timer <= self.stopping_time:
                            traci.vehicle.setSpeed(conflict_veh, 0)
                            traci.vehicle.setSpeedMode(conflict_veh, 0)
                            traci.vehicle.moveTo(conflict_veh, lane, lane_length)
                            self.timer += sp.TIME_STEP
                        else:
                            traci.vehicle.setSpeed(conflict_veh, -1)
                            traci.simulationStep()
                            new_acc = traci.vehicle.getAcceleration(conflict_veh)
                            self.control_unit.controller.acceleration = new_acc
                            self.control_unit.activator.current_acceleration = new_acc
                            self.timer = 0
                            self.stopping_lane = None

    def recolor_agents(self):
        """
        Recolours the agents using the colouring scheme set in SumoParameters.
        """
        shf.recolor_agent_list(traci.vehicle.getIDList(), sp.Colors().get_agent_colour())
        if self.control_unit.agents_in_view:
            shf.recolor_agent_list(self.control_unit.agents_in_view, sp.Colors().get_agent_colour("recognised"))
        if self.control_unit.activator.current_conflict_vehicle:
            shf.recolor_agent(self.control_unit.activator.current_conflict_vehicle, sp.Colors().get_agent_colour("critical"))
        if self.control_unit.agents_to_yield:
            shf.recolor_agent_list(self.control_unit.agents_to_yield, sp.Colors().get_agent_colour("yield"))
        if self.control_unit.agents_minor:
            shf.recolor_agent_list(self.control_unit.agents_minor, sp.Colors().get_agent_colour())

    def get_scenario_agent_by_id(self, id):
        for agent in self.scenario_agents:
            if agent.id == id:
                return agent

    def _get_lowest_data_length(self, new_agent):
        """
        Gets the minimum data length of the simulation agents, in order to determine the termination of the
        simulation.

        :param new_agent: the newly added simulation agent
        :type new_agent: Base.Agent
        """
        new_data_length = new_agent.trajectory.shape[0]
        if new_data_length < self.scenario_agents_data_length:
            self.scenario_agents_data_length = new_data_length

    def reset(self, crit_agent, ego_route, conflict_agent=None, conflict_route=None,):
        """
        Resets the current simulation to the starting parameters.

        :param crit_agent: the critical agent
        :type crit_agent: Base.Agent
        :param ego_route: the route of the ego agent
        :type ego_route: list[str]
        :param conflict_route: the route of the conflicting agent if present
        :type conflict_route: list[str]
        :param conflict_agent: the additional conflicting agent if present
        :type conflict_agent: Base.Agent
        """
        # --quit-on-end
        traci.close()
        shf.start_sumo()
        traci.route.add("ego_route", ego_route)
        if conflict_route: traci.route.add("conflict_route", conflict_route)

        self.control_unit.agent.reset()
        print(self.control_unit.agent.position.x, self.control_unit.agent.position.y)
        crit_agent.reset()

        # remove vehicles from simulation
        self.scenario_agents = []
        self.additional_sumo_agent = None
        self.current_data_step = 0
        self.stopping_pos = copy.copy(self._initial_stopping_pos)


        # read the agents
        self.control_unit.agent.add_to_sumo(sp.EGO_VEHICLE_ID, "ego_route")
        self.add_scenario_agent(crit_agent, crit_agent.id)

        if conflict_agent:
            conflict_agent.reset()
            self.add_sumo_agent(conflict_agent)

        traci.simulationStep()

    def run_simulation_without_av(self, ego_route, conflict_vehicle=None, conflict_route=None):
        """
        Runs the Simulation without the presence of an AV Agent.

        :param crit_vehicle: The critical agent for the current simulation
        :type crit_vehicle: Base.Agent
        :param ego_route: the edge list (route) of the ego agent
        :type ego_route: list[str]
        :param conflict_route: the edge list (route) of the conflicting agent
        :type conflict_route: list[str]
        :param conflict_vehicle: the additional conflict vehicle if one exists
        :type conflict_vehicle: Base.Agent
        :return: the ego trajectories for the simulation
        :rtype: dict
        """
        # Adds the conflicting Agent to the Simulator, which is always controlled by SUMO
        if conflict_vehicle: self.add_sumo_agent(conflict_vehicle)


        crit_vehicle = shf.create_agent_from_trajectory_data(
            os.path.join(self.control_unit.data_monitor.scenario_path, 'conflict_agent_data.csv'))
        self.add_scenario_agent(crit_vehicle, crit_vehicle.id)


        # Sets the current critical vehicle to be used within the stopping scenario
        self.control_unit.activator.current_conflict_vehicle = crit_vehicle.id
        self.control_unit.activator.current_conflict_vehicle = crit_vehicle.id
        sumo_agents = []
        running_times = dict()

        # Main control loop
        for type in self.ego_vehicle_types:

            # reset agents:
            self.reset(crit_vehicle, ego_route, conflict_vehicle, conflict_route)

            # Set the new vehicle type
            traci.vehicle.setType(self.control_unit.agent.id, typeID=type)
            self.control_unit.agent.cfm = type


            start_time = time.time()
            while not self.control_unit.arrived:

                if self.control_unit.activator.sim_agent_data_end():
                    self.control_unit.arrived = True
                    break

                # takes care of the conflict agent and the recording of their trajectories
                self.single_scenario_traci_step()

                # takes care of the ego vehicle and the trajectories
                self.control_unit.agent.update_trajectory()
                self.control_unit.agent.record_trajectory()

                traci.simulationStep()

            running_times[type] = time.time() - start_time
            self.control_unit.agent._change_recorded_trajectory_to_ndarray()
            sumo_agents.append(copy.deepcopy(self.control_unit.agent))
            self.control_unit.arrived = False

        for agent in sumo_agents:
            file_name = agent.cfm + "data"
            self.control_unit.data_monitor.safe_trajectory(agent, file_name)

        traci.close(False)
        return sumo_agents, running_times

    def set_ego_vehicle_types(self, type_list):
        """
        Sets the car following models to be evaluated for the current simulation.

        :param type_list: list containing all the car following models
        :type type_list: list[str]
        """
        self.ego_vehicle_types = type_list

    def create_results(self, sumo_agents):
        fig = plt.figure()
        ax = fig.add_subplot(221)
        bx = fig.add_subplot(222)
        cx = fig.add_subplot(223)
        dx = fig.add_subplot(224)

        ax.title.set_text("Trajectory")
        bx.title.set_text("Distance Autonomous - Conflict")
        cx.title.set_text("Speed")
        dx.title.set_text("Acceleration")

        # Plot the conflict agent
        conflict_agent = self.import_conflict_agent()
        self.scenario_agents_data_length = conflict_agent.trajectory.shape[0]
        conflict_agent.plot_dist_time(ax, color=sp.Colors().conflict_plot_data, label="Conflict Agent")
        conflict_agent.plot_speeds(cx, color=sp.Colors().conflict_plot_data, label="Conflict Agent")
        conflict_agent.plot_acceleration(dx, color=sp.Colors().conflict_plot_data, label="Conflict Agent")

        # Plot the sumo agents
        for agent in sumo_agents:
            # Plot the distance
            self.control_unit.agent = agent
            distances = self.control_unit.data_monitor.get_distances(conflict_agent)
            bx.plot(conflict_agent.trajectory[:, 0], distances, label=agent.cfm, linestyle='dashed')

            # Plot Trajectory
            agent.plot_dist_time(ax, label=agent.cfm, style='dashed')

            # Plot the speeds
            agent.plot_speeds(cx, label=agent.cfm, style='dashed')

            # Plot the accelerations
            agent.plot_acceleration(dx, label=agent.cfm, style='dashed')

        dist1 = self.control_unit.controller.calc_dist_to_pos((1764.22, 740.23),
                                                              origin=(1763.05, 872.32))
        dist2 = self.control_unit.controller.calc_dist_to_pos((1810, 874), origin=(1763.05, 872.32))
        ax.axhline(dist1, 0, 10, linestyle='dashed')
        ax.axhline(dist2, 0, 10, linestyle='dashed')

        # Plot the AV Agent:
        av_agent = self.import_av_data()
        conflict_agent = self.import_conflict_agent()
        self.control_unit.agent = av_agent
        self.scenario_agents_data_length = min(av_agent.trajectory.shape[0], conflict_agent.trajectory.shape[0])
        distances = self.control_unit.data_monitor.get_distances(conflict_agent)
        bx.plot(av_agent.trajectory[:, 0], distances, label="Autonomous Agent", color=sp.Colors().ego_plot_data)
        # Plot Trajectory
        av_agent.plot_dist_time(ax, label="Autonomous Agent", color=sp.Colors().ego_plot_data)


        # Plot the speeds
        av_agent.plot_speeds(cx, label="Autonomous Agent", color = sp.Colors().ego_plot_data)
        # Plot the accelerations
        av_agent.plot_acceleration(dx, label="Autonomous Agent", color = sp.Colors().ego_plot_data)

        ax.set_xlim(0, self.control_unit.agent.trajectory[-1, 0])
        bx.set_xlim(0, self.control_unit.agent.trajectory[-1, 0])
        cx.set_xlim(0, self.control_unit.agent.trajectory[-1, 0])
        dx.set_xlim(0, self.control_unit.agent.trajectory[-1, 0])

        dx.legend(loc='best')

        plt.show()
        print("test")

    def import_av_data(self):
        df = pd.read_csv(os.path.join(self.control_unit.data_monitor.scenario_path,
                                      'av_agent_data.csv'), usecols=['time', 'sim_time', 'TESIS_0_x', 'TESIS_0_y',
                                                                     'TESIS_0_speed', 'TESIS_0_angle'])

        return base.Agent(autonomous=False, sumo_data=df.values, type="autonomous")

    def import_conflict_agent(self):
        df = pd.read_csv(os.path.join(self.control_unit.data_monitor.scenario_path,
                                      'conflict_agent_data.csv'), usecols=['time', 'sim_time', 'TESIS_0_x', 'TESIS_0_y',
                                                                     'TESIS_0_speed', 'TESIS_0_angle'])

        return base.Agent(autonomous=False, sumo_data=df.values, type="bike")


class Helper:
    def __init__(self):
        self.param1 = None
        self.param2 = None
        self.param3 = None
        self.param4 = None
        self.on = True


