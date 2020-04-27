import os
import CustomSumoNetVis as SumoNetVis

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

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
        self.net = SumoNetVis.Net(NET_DIR)