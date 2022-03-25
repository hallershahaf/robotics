import csv
from Plotter import Plotter
import numpy as np
from shapely.geometry.polygon import Polygon, LineString
from typing import List, Tuple
import math
from matplotlib import pyplot as plt


def order_obs_points(points):
    first_point = points[0]
    for point in points[1:]:
        if point[1] < first_point[1] or (point[1] == first_point[1]) and (point[0] < first_point[0]):
            first_point = point
    new_order = [first_point]
    for i in range(len(points)-1):
        max_angle = 100  # math.atan2(points[i+1][1]-points[i][1], points[i+1][0]-points[i][0])
        # max_angle = max_angle if max_angle >= 0 else max_angle + np.pi
        # next_point = _
        for j in range(len(points)):
            if new_order[i] == points[j] or points[j] in new_order:
                continue
            curr_angle = np.arctan2(points[j][1]-new_order[i][1], points[j][0]-new_order[i][0]) % (2 * np.pi)
            if curr_angle < max_angle:
                max_angle = curr_angle
                next_point = points[j]
        new_order.append(next_point)
    return new_order


def is_counterclockwise_order(polygon: List[Tuple]) -> bool:
    return np.linalg.det(np.array(
        [[polygon[0][0], polygon[0][1], 1], [polygon[1][0], polygon[1][1], 1], [polygon[2][0], polygon[2][1], 1]])) >= 0


def first_point_have_minimum_y_cord(polygon: List[Tuple]) -> bool:
    first_point = polygon[0]
    for point in polygon[1:]:
        if point[1] < first_point[1] or (point[1] == first_point[1]) and (point[0] < first_point[0]):
            return False
    return True


def get_first_point(polygon: List[Tuple]) -> int:
    first_point = polygon[0]
    point_index = 0
    for i, point in enumerate(polygon):
        if point[1] < first_point[1] or (point[1] == first_point[1] and point[0] < first_point[0]):
            first_point = point
            point_index = i
    return point_index


def create_obstacle(points: List[Tuple]):
    points = order_obs_points(points)
    original_shape = Polygon(points)
    obstacle_points = list(original_shape.exterior.coords)
    first_index_point = get_first_point(obstacle_points)
    if not first_index_point == 0:
        obstacle_points = obstacle_points[first_index_point:] + obstacle_points[1:first_index_point] + [
            obstacle_points[first_index_point]]

    if not is_counterclockwise_order(obstacle_points):
        obstacle_points.reverse()
    # fig = plt.figure()
    # ax = fig.add_subplot()
    # ax.add_patch(plt.Polygon(Polygon(obstacle_points).exterior.coords, color='b'))
    # plt.autoscale()
    # plt.draw()  # draw the plot
    # plt.pause(1)
    return Polygon(obstacle_points)

file = open(
    'D:\Technion\Computer Science\Winter 2021-22\Robotics\Project\OneDrive_2022-01-16\Project Material\Simulator\I2R_Sim_WM_640x480\I2R_Sim_WM_640x480\obstacles_100m_above_sea_level.csv')
csvreader = csv.reader(file)
header = []
header = next(csvreader)
print(header)
rows = []
for row in csvreader:
    rows.append(row)
print(rows)
file.close()

obstacles = {}
obs_id = rows[0][4]
points = []

count = 1
for row in rows:
    if not obs_id == row[4]:
        obstacles[obs_id] = create_obstacle(points)
        points = []
        obs_id = row[4]
        points.append((int(row[1]), int(row[2])))
        count += 1
        print(count)
    else:
        points.append((int(row[1]), int(row[2])))
obstacles[obs_id] = create_obstacle(points)

# for obstacles in obstacles.values():
#     print(list(obstacles.exterior.coords)[:-1])
plotter1 = Plotter()
plotter1.add_obstacles(list(obstacles.values()))
plotter1.show_graph()


