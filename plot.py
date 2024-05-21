import math

import matplotlib.pyplot as plt

ax: plt.Axes
fig, ax = plt.subplots()
ax.set_ylim([0, 3])
ax.set_xlim([0, 3])

CLEARANCE = 0.20


def distance(xy1: tuple[float, float], xy2: tuple[float, float]) -> float:
    return math.sqrt((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2)


def rotate90(xy: tuple[float, float]) -> tuple[float, float]:
    return -xy[1], xy[0]


def get_direction(xy1: tuple[float, float], xy2: tuple[float, float]) -> tuple[float, float]:
    x_dif = xy2[0] - xy1[0]
    y_dif = xy2[1] - xy1[1]
    dis = distance(xy1, xy2)
    return x_dif / dis, y_dif / dis


def get_new_point(xy: tuple[float, float], direction: tuple[float, float], distance: float) -> tuple[float, float]:
    return xy[0] + direction[0] * distance, xy[1] + direction[1] * distance


def plot_line(xy1: tuple[float, float], xy2: tuple[float, float], color='blue'):
    x = [xy1[0], xy2[0]]
    y = [xy1[1], xy2[1]]
    ax.plot(x, y, color=color)


def plot_track(xy1: tuple[float, float], xy2: tuple[float, float]):
    plot_line(xy1, xy2)

    dir = get_direction(xy1, xy2)
    rot = rotate90(dir)
    plot_line(get_new_point(xy1, rot, CLEARANCE), get_new_point(xy2, rot, CLEARANCE), color='red')
    rot = rotate90(rotate90(rot))
    plot_line(get_new_point(xy1, rot, CLEARANCE), get_new_point(xy2, rot, CLEARANCE), color='red')


plot_track((0.2, 1.0), (2.5, 1.5))
plot_track((1.0, 1.0), (1.0, 2.5))

plt.show()
