import math
import time
from math import radians, sin

import matplotlib as mpl
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from scipy.interpolate import make_interp_spline
from shapely.geometry import LineString



class Colours:
    def __init__(self):
        self.red = (255, 0, 0)
        self.light_red = (255, 153, 153, 150)
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.light_green = (0, 153, 0)
        self.yellow = (255, 255, 102)
        self.grey = (160, 160, 160)
        self.building = '#887c7a' #grey
        self.mint = (0, 204, 204)
        self.light_blue = (102, 178, 255)
        self.tree_start_nd = '#ff3333' # light red
        self.tree_rand_nd = '#9933ff'   # light yellow
        self.tree_nearest_nd = '#99ff33' # light green
        self.tree_new_nd = '#ff3399'    # pink
        self.tree_nd = '#3399ff' # light blue
        self.mpl_line_blue = '#88aaff' # blue
        self.mpl_line_purple = '#581845' # purple
        self.mpl_line_red = '#c70039' # red
        self.mpl_line_green = '#DAF7A6' # green




# ------------------------ DEFINES ----------------------
color = Colours()
# TIME = da.Timing()
# -------------------------------------------------------



def clothoid_ode_rhs(state, s, kappa0, kappa1):
    x, y, theta = state[0], state[1], state[2]
    return np.array([np.cos(theta), np.sin(theta), kappa0 + kappa1 * s])


def eval_clothoid(x0, y0, theta0, kappa0, kappa1, s):
    return odeint(clothoid_ode_rhs, np.array([x0, y0, theta0]), s, (kappa0, kappa1))

def last(args, value):
    """ writes value to the last entry of a 1D array """
    args[np.shape(args)[0] - 1] = value
    return args

def print_turning_radius(step_length, max_steering, car_length):
    x_values = np.zeros((1, 1))
    for i in range(step_length, max_steering + step_length, step_length):
        x_values = np.append(x_values, i)
    len = np.shape(x_values)[0]

    y_values = np.zeros((1, 1))
    for i in range(1, len):
        turning_radius = car_length / sin(radians(x_values[i]))
        y_values = np.append(y_values, turning_radius)

    x_smooth, y_smooth = smoothen(x_values, y_values, k=9)
    len = np.shape(x_smooth)[0]

    fig, ax = plt.subplots()
    #ax.plot(x_values[1: len], y_values[1: len])
    ax.plot(x_smooth[step_length * 4:len], y_smooth[step_length *4:len])

    ax.set(xlabel='Steering Angle [°]', ylabel='Turning Radius [m]',
           title='Turning Radius in Relation to the Steering Angle')
    ax.grid()

    plt.show()


def smoothen(x_old, y_old, k, steps=300):
    """ smoothens a line-plot using a Bspline """
    x_smooth = np.linspace(x_old.min(), x_old.max(), steps)
    spl = make_interp_spline(x_old, y_old, k=k)
    y_smooth = spl(x_smooth)
    return x_smooth, y_smooth


def rotate(origin, point, angle):
    """ Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians. """
    ox, oy = origin[0], origin[1]
    px, py = point[0], point[1]

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return np.array([qx, qy])


def get_rectangle_lines(rect):
    """ returns the four sides of a rect as with their start and end points """
    x = rect.x
    y = rect.y
    width = rect.width
    height = rect.height
    top = (x, y, x + width, y)
    bottom = (x + width, y + height, x, y + height)
    left = (x, y + height, x, y)
    right = (x + width, y, x + width, y + height)
    return [top, right, bottom, left]


def plot_line_stuff(line, ax):
    line_x = (line[0], line[2])
    line_y = (line[1], line[3])
    ax.plot(line_x, line_y)


def plot_building(ax, rect, clr=color.building):
    """ adds a building to the current scenario plot. """
    rect = patches.Rectangle((rect[0], rect[1]), rect[2], rect[3], linewidth=1, edgecolor=clr, facecolor=clr)
    ax.add_patch(rect)


def plot_car_bounding_box(ax, car, game):
    """ This function plots the cars bounding box into an existing plot. """
    ts = ax.transData
    coords = [car.position.x * game.ppu, car.position.y * game.ppu]
    tr = mpl.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], -car.angle)
    t = tr + ts

    rect = patches.Rectangle((car.position.x * game.ppu - car.px_length / 2, car.position.y * game.ppu -
                              car.px_width / 2), car.px_length, car.px_width, linewidth=1, color="blue", transform=t)

    rect.set_transform(t)
    ax.add_patch(rect)


def calc_point_dist(pt1, pt2):
    """ Calculates the distance between two points. """
    return math.sqrt((pt2.y - pt1.y) ** 2 + (pt2.x - pt1.x) ** 2)


def plot_point_list(pt_list, marker_style='rx'):
    fig, ax = plt.subplots()
    ax.set(xlabel='x', ylabel='y',
           title='Point Plot')
    ax.grid()

    for pt in pt_list:
        ax.plot(pt.x, pt.y, marker_style)

    plt.axis('equal')
    # invert the y_axis
    plt.gca().invert_yaxis()
    plt.show()


def pt_within_rect(rect, pt):
    """ Checks whether a point lies within a rectangle. Rect is given as a Vector2-List with the points clockwise,
        starting with the top left corner. """
    for i in range(0, 4):
        if i == 3:
            if not is_right_of_line(rect[i], rect[0], pt):
                return False
        elif not is_right_of_line(rect[i], rect[i + 1], pt):
            return False
    return True


def is_right_of_line(pt_a, pt_b, pt_x):
    ret_val = ((pt_b.x - pt_a.x) * (pt_x.y - pt_a.y) - (pt_b.y - pt_a.y) * (pt_x.x - pt_a.x)) > 0
    return ret_val


def line_rect_intersect(line, rect):
    """ Calculates all the intersections of a sensor ray and a building. """
    line = LineString([(line[0], line[1]), (line[2], line[3])])
    rect_boundary = get_rectangle_lines(rect)
    for wall in rect_boundary:
        wall_line = LineString([(wall[0], wall[1]), (wall[2], wall[3])])
        if line.intersection(wall_line):
            return True
    return False


def pt_in_circle(centre, radius, pt):
    if calc_point_dist(centre, pt) < radius:
        return True
    else:
        return False


def timing(f):
    def wrap(*args):
        time1 = time.time()
        ret = f(*args)
        time2 = time.time()
        print('{:s} function took {:.3f} ms'.format(f.__name__, (time2-time1)*1000.0))

        return ret
    return wrap


def quadrant(pt, centre):
    if (pt.x > centre.x and pt.y < centre.y):
        return 1
    elif (pt.x < centre.x and pt.y > centre.y):
        return 2
    elif (pt.x < centre.x and pt.y < centre.y):
        return 3
    elif (pt.x > centre.x and pt.y > centre.y):
        return 4
    elif (pt.x == centre.x and pt.y < centre.y):
        return 1
    elif (pt.x == centre.x and pt.y > centre.y):
        return 3
    elif (pt.y == centre.y and pt.x > centre.x):
        return 2
    elif (pt.y == centre.y and pt.x < centre.x):
        return 1
    else:
        return None



if __name__ == "__main__":
    x0, y0, theta0 = 0, 0, 0
    L = 6
    kappa0, kappa1 = 0, -.04
    s = np.linspace(0, L, 100)
    overtaking_length = 5
    complete_x = []
    complete_y = []

    # 1. Clothoid
    sol = eval_clothoid(x0, y0, theta0, kappa0, kappa1, s)
    xs, ys, thetas = sol[:, 0], sol[:, 1], sol[:, 2]
    complete_x.extend(xs)
    complete_y.extend(ys)

    # 2. Clothoid
    x_vals = []
    y_vals = []
    for i in range(len(xs) - 1, 0, -1):
        new_pt = rotate([xs[-1], ys[-1]], [xs[i], ys[i]], math.pi)
        x_vals.append(new_pt[0])
        y_vals.append(new_pt[1])
    complete_x.extend(x_vals)
    complete_y.extend(y_vals)

    # Straight Bit
    x_vals = [x_vals[-1]]
    y_vals = [y_vals[-1]]
    for i in range(1, overtaking_length + 1):
        x_vals.append(x_vals[-1] + 1)
        y_vals.append(y_vals[-1])
    complete_x.extend(x_vals)
    complete_y.extend(y_vals)

    # 3. Clothoid
    sol = eval_clothoid(x_vals[-1], y_vals[-1], 0, 0, -kappa1, s)
    xs, ys, thetas = sol[:, 0], sol[:, 1], sol[:, 2]
    complete_x.extend(xs)
    complete_y.extend(ys)

    # 4. Clothoid
    x_vals = []
    y_vals = []
    for i in range(len(xs) - 1, 0, -1):
        new_pt = rotate([xs[-1], ys[-1]], [xs[i], ys[i]], math.pi)
        x_vals.append(new_pt[0])
        y_vals.append(new_pt[1])
    complete_x.extend(x_vals)
    complete_y.extend(y_vals)

    # Create Complete Path
    complete = []
    for i in range(0, len(complete_x)):
        complete.append((complete_x[i], complete_y[i]))

    plt.axis('equal')
    plt.plot(complete_x, complete_y)
    plt.show()
    rotated_x = []
    rotated_y = []
    rotated = []
    for i in range(0, len(complete)):
        new_pt = rotate(complete[0], complete[i], 90)
        rotated.append(new_pt)

    test = LineString(rotated)
    print("done")
    #
    # return LineString(complete)



