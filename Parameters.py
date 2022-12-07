import os
from math import radians, sin

import numpy as np

import CustomSumoNetVis as SumoNetVis
import HelperFcts as hf

# ------------------------ DEFINES ----------------------
color = hf.Colours()
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
# -------------------------------------------------------


class SETTINGS:
    """
    Main Settings Class, storing all the settings for the Pygame Simulation
    """
    def __init__(self, machine="Laptop", sumo_on=False):
        """
        Settings Class Constructor, needs to be used in every file accessing the Pygame Simulation

        :param machine: specifies the machine, where the simulation is run on and changes the window sizes etc.
        :type machine: str
        :param sumo_on: states, whether to connect the simulation to SUMO or not (True - connected)
        :type sumo_on: bool
        """
        self.ppu = 64
        self.ticks = 60
        self.background_color = color.white
        self.sumo_on = sumo_on
        # define window start position on screen
        window_pos_x = 0
        window_pos_y = 31
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (window_pos_x, window_pos_y)

        # define window width etc.
        self.win_width = 0
        self.win_height = 0

        if machine == "Home-PC":
            self.button_size = 100
            self.dsg_car_width = 250
            self.mode = "Home-PC"
            if self.sumo_on:
                self.win_width = 1280
                self.win_height = 1368
                self.dsg_width = 1280
                self.dsg_height = 1368
            else:
                self.win_width = 1920
                self.win_height = 1080
                self.dsg_width = 1920
                self.dsg_height = 1080
        elif machine == "Laptop":
            self.button_size = 50
            self.dsg_car_width = 100
            self.mode = "Laptop"
            if self.sumo_on:
                self.win_width = 650
                self.win_height = 650
                self.dsg_width = 650
                self.dsg_height = 650
            else:
                self.win_width = 800
                self.win_height = 600
                self.dsg_width = 800
                self.dsg_height = 600


class AGENT_PARAMS1:
    """ Class to store the agent's parameters such as width, length, speed etc. """
    def __init__(self, desired_rad=None, factor=None):
        """
        Agent Class Constructor

        :param desired_rad: If this values is given, the length and the width of the agent are calculated based on the
                            desired turning radius. A scaling factor is also returned, which should be used in other
                            drawing actions
        :type desired_rad: float
        :param factor: Instead of a desired turning radius a scaling factor can be given, scaling the length and width
                       of the agent by the given factor
        :type factor: float
        """
        self.width = 2.33
        self.length = 5
        self.reaction_time = 2.0
        self.max_acceleration = 10
        self.max_steering = 65
        self.free_deceleration = 20.0
        self.max_velocity = 20
        self.brake_deceleration = 10
        self.sensor_surface_dims = 3000
        self.max_turning_radius = self.length / sin(radians(self.max_steering))
        self.max_curvature = 1 / self.max_turning_radius
        self.scaling_factor = 1

        if factor is not None or desired_rad is not None:
            self.do_scaling(desired_rad, factor)

    def get_parameters(self):
        """
        Returns the main agent parameters as tuple

        :return: The main agent parameters
        :rtype: tuple[float, float, float, float, float, float, float, float, int]
        """
        return self.width, self.length, self.reaction_time, self.max_acceleration, self.max_steering, \
               self.free_deceleration, self.max_velocity, self.brake_deceleration, self.sensor_surface_dims

    def do_scaling(self, desire_rad=None, factor=None):
        """
        This function is called, if a desired radius or scaling factor is given for the agent. It scales all the related
        parameters.

        :param desire_rad: the desired turning radius for the agent
        :type desire_rad: float
        :param factor: the scaling factor of the agent: width -> width * scaling factor
        :type factor: float
        """
        if desire_rad is not None:
            orig_rad = self.max_turning_radius
            fac =  desire_rad / orig_rad
            self.max_turning_radius = desire_rad
            self.max_curvature = 1 / self.max_turning_radius
            self.length = fac * self.length
            self.width = fac * self.width
            self.scaling_factor = fac
        if factor:
            self.length = factor * self.length
            self.width = factor * self.width
            self.max_turning_radius = self.length / sin(radians(self.max_steering))
            self.max_curvature = 1 / self.max_turning_radius
            self.scaling_factor = factor

    def get_speed(self, current_speed, dt):
        """
        Gets the agent's speed for the next time step given a certain time increment

        :param current_speed: the agent's current speed
        :type current_speed: Vector2
        :param dt: time increment
        :type dt: float
        :return: the agent's speed for the next time-step
        :rtype: Vector2
        """
        new_speed = np.array((1, 2))
        if current_speed.x < self.max_velocity:
            new_speed.x = current_speed.x + self.max_acceleration * dt
        else:
            new_speed.x = current_speed.x
        return new_speed


class LIDAR_PARAMS1:
    """ Class to store some LIDAR Sensor parameters.  """
    def __init__(self):
        """
        Main LIDAR 1 Settings Constructor
        """
        self.direction = 90
        self.reach = 150 # [m]
        self.px_reach = int(self.reach * 18.8)
        self.angle = 18
        self.cover = 20

    def get_parameters(self):
        """
        Returns the main LIDAR sensor parameters

        :return: sensor parameters
        :rtype: tuple[float, float, float]
        """
        return self.px_reach, self.angle, self.cover


class LIDAR_PARAMS2:
    """ Class to store some LIDAR Sensor parameters.  """
    def __init__(self):
        """
        Main LIDAR 2 Settings Constructor
        """
        self.direction = 90
        self.reach = 60 # [m]
        self.px_reach = int(self.reach * 18.8)
        self.angle = 56
        self.cover = 20

    def get_parameters(self):
        """
        Returns the main LIDAR sensor parameters

        :return: sensor parameters
        :rtype: tuple[float, float, float]
        """
        return self.px_reach, self.angle, self.cover


class CITY1:
    """
    Main Class to store the settings for a city object within the Pygame Simulation.
    """
    def __init__(self, settings):
        """
        Main City Constructor based on the project settings in order to create the correct pygame surfaces

        :param settings: The simulation settings
        :type settings: SETTINGS
        """
        self.cell_nbr = 10
        self.bldg_width = int(settings.win_width / self.cell_nbr)
        self.bldg_height = int(settings.win_height / self.cell_nbr)
        self.bldg_mtx = np.zeros((self.cell_nbr, self.cell_nbr))
        self.bldg_pos = []
        self.bldg_mtx[0, :] = 1
        self.bldg_mtx[-1, :] = 1
        self.settings = settings
        self.generate_bldg_positions()

    def generate_bldg_positions(self):
        """
        This function creates random buildings over the simulations surface.
        """
        for i in range(0, np.shape(self.bldg_mtx)[0]):
            for j in range(0, np.shape(self.bldg_mtx)[1]):
                if self.bldg_mtx[i, j] == 1:
                    pos_x = j * self.bldg_width + self.bldg_width / 2
                    pos_y = self.settings.win_height - (i * self.bldg_height + self.bldg_height / 2)
                    self.bldg_pos.append(np.array([pos_x, pos_y]))


class NETWORK:
    """ Class to store the SUMO Network """
    def __init__(self, name):
        """
        Loads the given SUMO network using SumoNetVis

        :param name: Name of the SUMO network file
        :type name: str
        """
        SUMO_DIR = os.path.join(ROOT_DIR, 'Sumo/Files')
        NET_DIR = os.path.join(SUMO_DIR, name)
        agent_settings = AGENT_PARAMS1(desired_rad=30)
        SumoNetVis.DEFAULT_LANE_WIDTH = agent_settings.scaling_factor * SumoNetVis.DEFAULT_LANE_WIDTH
        self.net = SumoNetVis.Net(NET_DIR)


def get_city1(settings):
    """
    Returns a city with the respective obstacle matrix

    :param settings: the simulation settings
    :type settings: SETTINGS
    :return: returns the position of the buildings in a matrix and the respective building dimensions
    :rtype: tuple[np.ndarray, float, float]
    """
    if settings.mode == "Laptop":
        bldg_width = 220
    elif settings.mode == "Home-PC":
        bldg_width = 480
    bldg_height = settings.win_height / settings.win_width * bldg_width
    cell_nbr_x = round(settings.win_width / bldg_width) * 2
    cell_nbr_y = round(settings.win_height / bldg_height) * 2
    bldg_width = settings.win_width / cell_nbr_x
    bldg_height = settings.win_height / cell_nbr_y
    bldg_mtx = np.zeros((cell_nbr_x, cell_nbr_y))
    bldg_mtx[0, :] = np.array([1, 1, 0, 1, 1, 0, 1, 1])
    bldg_mtx[1, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[2, :] = np.array([1, 1, 1, 1, 0, 1, 1, 0])
    bldg_mtx[3, :] = np.array([1, 1, 1, 1, 0, 1, 1, 0])
    bldg_mtx[4, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[5, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[6, :] = np.array([1, 1, 0, 1, 0, 1, 1, 1])
    bldg_mtx[7, :] = np.array([1, 1, 0, 1, 0, 1, 1, 1])

    return bldg_mtx, bldg_width, bldg_height


def get_city2(settings):
    """
    Returns a city with the respective obstacle matrix

    :param settings: the simulation settings
    :type settings: SETTINGS
    :return: returns the position of the buildings in a matrix and the respective building dimensions
    :rtype: tuple[np.ndarray, float, float]
    """
    if settings.mode == "Laptop":
        bldg_width = 220
    elif settings.mode == "Home-PC":
        bldg_width = 480
    bldg_height = settings.win_height / settings.win_width * bldg_width
    cell_nbr_x = round(settings.win_width / bldg_width) * 2
    cell_nbr_y = round(settings.win_height / bldg_height) * 2
    bldg_width = settings.win_width / cell_nbr_x
    bldg_height = settings.win_height / cell_nbr_y
    bldg_mtx = np.zeros((cell_nbr_x, cell_nbr_y))
    bldg_mtx[0, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[1, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[2, :] = np.array([0, 1, 1, 0, 0, 1, 1, 0])
    bldg_mtx[3, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[4, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[5, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[6, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[7, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    return bldg_mtx, bldg_width, bldg_height


def get_city3(settings):
    """
    Returns a city with the respective obstacle matrix

    :param settings: the simulation settings
    :type settings: SETTINGS
    :return: returns the position of the buildings in a matrix and the respective building dimensions
    :rtype: tuple[np.ndarray, float, float]
    """
    if settings.mode == "Laptop":
        bldg_width = 220
    elif settings.mode == "Home-PC":
        bldg_width = 480
    bldg_height = settings.win_height / settings.win_width * bldg_width
    cell_nbr_x = round(settings.win_width / bldg_width) * 2
    cell_nbr_y = round(settings.win_height / bldg_height) * 2
    bldg_width = settings.win_width / cell_nbr_x
    bldg_height = settings.win_height / cell_nbr_y
    bldg_mtx = np.zeros((cell_nbr_x, cell_nbr_y))
    bldg_mtx[0, :] = np.array([0, 0, 0, 0, 1, 0, 0, 0])
    bldg_mtx[1, :] = np.array([0, 0, 0, 0, 1, 0, 0, 0])
    bldg_mtx[2, :] = np.array([0, 1, 1, 0, 1, 1, 0, 0])
    bldg_mtx[3, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[4, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[5, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[6, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    bldg_mtx[7, :] = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    return bldg_mtx, bldg_width, bldg_height


class IMG_PATHS:
    """ this class stores all the images used in the project """
    def __init__(self):
        """
        Main class constructor
        """
        MODELS_DIR = os.path.join(ROOT_DIR, 'models')

        self.settings_btn = os.path.join(MODELS_DIR, 'btn_settings.png')
        self.agent_img_large = os.path.join(MODELS_DIR, 'carmodel_blue_big.png')
        self.agent_img_small = os.path.join(MODELS_DIR, 'carmodel_blue_small.png')
        self.lidar_btn = os.path.join(MODELS_DIR, 'btn_lidar.png')
        self.camera_btn = os.path.join(MODELS_DIR, 'btn_camera.png')
        self.building_btn = os.path.join(MODELS_DIR, 'btn_building.png')
        self.building_img = os.path.join(MODELS_DIR, 'house_simple.png')
        self.signs_btn = os.path.join(MODELS_DIR, 'btn_signs.png')
        self.target_btn = os.path.join(MODELS_DIR, 'btn_target.png')
        self.steering_wheel = os.path.join(MODELS_DIR, 'steering_wheel.png')
        self.tumcar_small = os.path.join(MODELS_DIR, 'tumcar_grey_small.png')
        # self.agent = os.path.join(MODELS_DIR, 'model_rotated.png')
        self.agent = os.path.join(MODELS_DIR, 'car_grey2.png')
        self.conflict_bike = os.path.join(MODELS_DIR, 'conflict_cyclist.png')
        self.critical_bike = os.path.join(MODELS_DIR, 'critical_cyclist.png')
        self.default_cyclist = os.path.join(MODELS_DIR, 'default_cyclist.png')
        self.yield_bike = os.path.join(MODELS_DIR, 'agent_to_yield.png')
        self.marker_low = os.path.join(MODELS_DIR, 'marker_low.png')
        self.marker_low_to_medium = os.path.join(MODELS_DIR, 'marker_low_to_medium.png')
        self.marker_medium = os.path.join(MODELS_DIR, 'marker_medium.png')
        self.marker_medium_to_high = os.path.join(MODELS_DIR, 'marker_medium_to_high.png')
        self.marker_high = os.path.join(MODELS_DIR, 'marker_high.png')
        self.tree1 = os.path.join(MODELS_DIR, 'tree1.png')
        self.tree2 = os.path.join(MODELS_DIR, 'tree2.png')
        self.tree3 = os.path.join(MODELS_DIR, 'tree3.png')
        self.tree4 = os.path.join(MODELS_DIR, 'tree4.png')

class SUMO_PATHS:
    """This class stores all the paths to the SUMO files used for the simulation. """
    def __init__(self):
        """
        Main class constructor
        """
        self.config_file = os.path.join(ROOT_DIR, 'Sumo/Files/Simulation.sumocfg')
        self.exe = "C:/Users/Felix/OneDrive - Technische Universitat Munchen/Masterarbeit/SumoBuild/" \
                   "sumo-gui.exe"
        self.network = os.path.join(ROOT_DIR, 'Sumo/Files/ValidationNetwork.net.xml')

        self.scenarios = os.path.join(ROOT_DIR, 'Sumo/Scenarios')

        self.images = os.path.join(ROOT_DIR, 'Figures')


