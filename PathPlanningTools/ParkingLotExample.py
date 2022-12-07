import copy
from math import pi

import matplotlib
import matplotlib.patches
import matplotlib.pyplot as plt
import matplotlib.transforms
import numpy as np
from matplotlib.path import Path
from shapely.geometry import LineString, Polygon

import Paths as params
import PathPlanningTools.MyRRT as planner
import Sumo.SumoParameters as sp
from PathPlanningTools import Base


class ParkingLot:
    def __init__(self, height, width):
        self.height = height
        self.width = width
        self.space_width = 2.5
        self.space_height = 5
        self.boundary = None
        self.color = (.63, .63, .63, .4)
        self.parking_area = self.make_parking_area()
        self.parking_spots = [[],[]]
        self.taken = []
        self.cars = []
        self.dist = .5
        self.configuration_rects = []

    def make_parking_area(self):
        self.boundary = LineString([(0, 0), (0, self.height), (self.width, self.height), (self.width, 0), (0, 0)])
        return matplotlib.patches.Polygon(self.boundary.coords, True, color=self.color)

    def plot(self, ax=None):
        if not ax:
            fig = plt.figure()
            ax = plt.subplot(111)
        
        ax.add_patch(self.parking_area)
        self.plot_markings(ax)
        self.plot_cars(ax)
        self.plot_configuration_space(ax)
        plt.axis('equal')
        plt.pause(.1)

    def add_parking_row(self, start_pos, orientation, nbr_of_spots):
        pos = np.array(start_pos)
        for i in range(0, nbr_of_spots):
            self.add_single_spot(copy.copy(pos), orientation)
            pos[0] = pos[0] + self.space_width

    def add_single_spot(self, position, orientation):
        self.parking_spots[0].append(position)
        self.parking_spots[1].append(orientation)

    def plot_markings(self, ax):
        for i in range(0, len(self.parking_spots[0])):
            self.plot_single_space(self.parking_spots[0][i], self.parking_spots[1][i], ax)


    def plot_single_space(self, position, orientation, ax):

        boundary_x = [position[0]-self.space_width / 2,
                    position[0] - self.space_width / 2,
                    position[0] + self.space_width / 2,
                    position[0] + self.space_width / 2]
        if orientation == "s":
            boundary_y = [position[1] - self.space_height / 2,
                          position[1] + self.space_height / 2,
                          position[1] + self.space_height / 2,
                          position[1] - self.space_height / 2]
        else:
            boundary_y = [position[1] + self.space_height / 2,
                          position[1] - self.space_height / 2,
                          position[1] - self.space_height / 2,
                          position[1] + self.space_height / 2]

        ax.plot(boundary_x, boundary_y, linewidth=2, color=(1, 1, 1))

    def randomize_parking(self):
        for i in range(0, len(self.parking_spots[0])):
            self.taken.append(np.random.randint(0, 2))

    def plot_cars(self, ax):
        for i in range(0, len(self.taken)):
            if self.taken[i]:
                image = plt.imread(params.IMG_PATHS().agent)
                img = ax.imshow(image, origin="lower", zorder=10)
        
                scale = sp.CAR_WIDTH / image.shape[0]
                tscale = matplotlib.transforms.Affine2D().scale(scale)
        
                tx1 = sp.CAR_LENGTH / 2
                ty1 = sp.CAR_WIDTH / 2
                ttranslate1 = matplotlib.transforms.Affine2D().translate(-tx1, -ty1)
                
                if np.random.randint(0, 2):
                    angle = 0
                else:
                    angle = 180
                
                tr = matplotlib.transforms.Affine2D().rotate_deg(angle + 90)
        
                tx = self.parking_spots[0][i][0]
                ty = self.parking_spots[0][i][1]
                ttranslate2 = matplotlib.transforms.Affine2D().translate(tx, ty)
        
                trans_data = tscale + ttranslate1 + tr + ttranslate2 + ax.transData
                img.set_transform(trans_data)

                rect = LineString([(tx - sp.CAR_WIDTH/2, ty - sp.CAR_LENGTH/2),
                                   (tx + sp.CAR_WIDTH/2, ty - sp.CAR_LENGTH/2),
                                   (tx + sp.CAR_WIDTH/2, ty + sp.CAR_LENGTH/2),
                                   (tx -sp.CAR_WIDTH/2, ty * sp.CAR_LENGTH/2),
                                   (tx - sp.CAR_WIDTH / 2, ty - sp.CAR_LENGTH / 2)])
                self.cars.append(rect)

                configuration_rect = Polygon([(tx - sp.CAR_WIDTH / 2 - self.dist, ty - sp.CAR_LENGTH / 2 - self.dist),
                                                 (tx + sp.CAR_WIDTH / 2 + self.dist, ty - sp.CAR_LENGTH / 2 - self.dist),
                                                 (tx + sp.CAR_WIDTH / 2 + self.dist, ty + sp.CAR_LENGTH / 2 + self.dist),
                                                 (tx - sp.CAR_WIDTH / 2 - self.dist, ty + sp.CAR_LENGTH / 2 + self.dist),
                                                 (tx - sp.CAR_WIDTH / 2 - self.dist, ty - sp.CAR_LENGTH / 2 - self.dist)])
                self.configuration_rects.append(configuration_rect)

    def plot_configuration_space(self, ax):

        for rect in self.configuration_rects:
            point_list = []
            for pt in rect.exterior.coords:
                point_list.append((pt[0], pt[1]))
            path = Path(point_list)
            patch = matplotlib.patches.PathPatch(path, facecolor=(1, 0, 0, .5), lw=0)
            ax.add_patch(patch)

    def get_random_spot(self):
        for i in range(0, len(self.taken)):
            if not self.taken[i]:
                if np.random.randint(0, 2):
                    return Base.Node(self.parking_spots[0][i][0], self.parking_spots[0][i][1], pi / 2)


if __name__ == "__main__":
    parking = ParkingLot(50, 45)
    parking.add_parking_row((11.25, 12.5), "s", 10)
    parking.add_parking_row((11.25, 17.5), "n", 10)
    parking.add_parking_row((11.25, 32.5), "s", 10)
    parking.add_parking_row((11.25, 37.5), "n", 10)
    parking.randomize_parking()

    fig = plt.figure()
    ax = plt.subplot(111)

    # plt.axis('equal')
    parking.plot(ax)
    plt.xlim(0, parking.width)
    plt.ylim(0, parking.height)
    plt.pause(.1)

    parking_entry = Base.Node(0, 25, 0)
    desired_spot = parking.get_random_spot()

    plan_alg = planner.RRT(parking_entry, parking.width, 10000, None)
    plan_alg.search_space = (parking.width, parking.height)
    plan_alg.set_obstacles(parking.configuration_rects)
    plan_alg.set_obstacle_function(plan_alg.obstacle_free2)
    plan_alg.set_sample_func(plan_alg.search_space_sample)
    plan_alg.rrt_star(parking_entry, desired_spot, 1, 1, ball_radius=2, ax=ax)

    # plan_alg = planner.DubinsRRT(parking_entry, parking.width, 10000, None)
    # plan_alg.search_space = (parking.width, parking.height)
    # plan_alg.set_obstacles(parking.configuration_rects)
    # plan_alg.set_obstacle_function(plan_alg.obstacle_free2)
    # plan_alg.set_sample_func(plan_alg.search_space_sample)
    # plan_alg.rrt_star(parking_entry, desired_spot, 1.5, .5, ball_radius=5, ax=ax)

    # plan_alg.rrt_tree.plot(ax)
    print("Done")

