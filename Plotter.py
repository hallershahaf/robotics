from typing import List, Tuple

import matplotlib.lines as lines
from matplotlib import pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon as plotpol
from shapely.geometry.polygon import Polygon, LineString


class Plotter:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()
        self.ax.invert_xaxis()
        plt.grid()

    def add_obstacles(self, obstacles: List[Polygon]):
        for obstacle in obstacles:
            self.ax.add_patch(plt.Polygon(obstacle.exterior.coords, alpha=0.5, color='b'))

    def add_robot(self, point, distance_to_vertex):
        x, y = point
        self.distance_to_vertex = distance_to_vertex
        target = Polygon([(x - distance_to_vertex, y - distance_to_vertex), (x + distance_to_vertex, y - distance_to_vertex), (x + distance_to_vertex, y + distance_to_vertex), (x - distance_to_vertex, y + distance_to_vertex)])
        self.ax.add_patch(plt.Polygon(target.exterior.coords, color='g'))

    def add_c_space_obstacles(self, obstacles: List[Polygon]):
        for obstacle in obstacles:
            self.ax.add_patch(plt.Polygon(obstacle.exterior.coords, color='r', alpha=0.5))

    def add_visibility_graph(self, edges: List[LineString]):
        for edge in edges:
            plt.plot(list(edge.xy[0]), list(edge.xy[1]), color='black', linestyle='dashed', linewidth=0.1)

    def add_shorterst_path(self, edges):
        if len(edges) > 0:
            current_vertex = edges[0]
            for edge in edges[1:]:
                plt.plot([current_vertex[0], edge[0]], [current_vertex[1], edge[1]], color='yellow', linewidth=1,
                        alpha=0.4)
                self.add_robot(edge, self.distance_to_vertex)
                current_vertex = edge
    
    def add_lidar_points(self, points):
        for point in points:
            self.ax.plot(point[0], point[1], markersize=1.0, c='b', marker='o')
            #plt.pause(0.0005)

    def add_drone_points(self, points):
        for point in points:
            self.ax.plot(point[0], point[1], markersize=1.4, c='r', marker='o')
            #plt.pause(0.0005)     

    def show_graph(self):
        plt.autoscale()
        #self.ax.invert_xaxis()
        self.ax.set_axisbelow(True)
        plt.title('Height = -100[m]')
        plt.xlabel('X[m]')
        plt.ylabel('Y[m]')
        #plt.draw()  #draw the plot
        #plt.pause(10)
        plt.show()
