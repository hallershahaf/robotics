import argparse
import itertools
import os
import typing

from queue import PriorityQueue

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString, Point
from shapely.geometry import mapping
import numpy as np
import time
from matplotlib import pyplot as plt


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


def is_counterclockwise_order(polygon: typing.List[typing.Tuple]) -> bool:
    """
    :param polygon: List of vertices' coordinates of a convex polygon.
    :return: True if the vertices are ordered in counter-clockwise order, false otherwise.
    """
    if len(polygon) < 3:
        return True
    return np.linalg.det(np.array(
        [[polygon[0][0], polygon[0][1], 1], [polygon[1][0], polygon[1][1], 1], [polygon[2][0], polygon[2][1], 1]])) >= 0


def get_first_point(polygon: typing.List[typing.Tuple]) -> int:
    """
    :param polygon: List of vertices' coordinates of a convex polygon.
    :return: Returns the index of the vertex that has the minimal y coordinate, and has the minimal x coordinate of all
    vertices attaining the minimal y coordinate.
    """
    first_point = polygon[0]
    point_index = 0
    for i, point in enumerate(polygon):
        if point[1] < first_point[1] or (point[1] == first_point[1] and point[0] < first_point[0]):
            first_point = point
            point_index = i
    return point_index


# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """

    robot_points = [(-r, -r), (r, -r), (r, r), (-r, r), (-r, -r)]
    # robot_points = [(0, -r), (r, 0), (0, r), (-r, 0), (0, -r)]

    obstacle_points = list(
        original_shape.exterior.coords)  # Assumption: the first point have the smallest y-coordinate?
    first_index_point = get_first_point(obstacle_points)
    if not first_index_point == 0:
        obstacle_points = obstacle_points[first_index_point:] + obstacle_points[1:first_index_point] + [
            obstacle_points[first_index_point]]

    if not is_counterclockwise_order(obstacle_points):
        obstacle_points.reverse()
    new_shape = []

    i, j = 0, 0
    while True:
        # print("i: {}, j: {}".format(i, j))
        new_shape.append(tuple(map(lambda x_1, x_2: x_1 + x_2, robot_points[i], obstacle_points[j])))

        # Calculate the angles
        robot_angle = None
        if i < len(robot_points) - 1:
            vector_robot = [robot_points[i + 1][0] - robot_points[i][0], robot_points[i + 1][1] - robot_points[i][1]]
            robot_angle = np.arctan2(vector_robot[1], vector_robot[0]) % (2 * np.pi)
        obstacle_angle = None
        if j < len(obstacle_points) - 1:
            vector_obstacle = [obstacle_points[j + 1][0] - obstacle_points[j][0],
                               obstacle_points[j + 1][1] - obstacle_points[j][1]]
            obstacle_angle = np.arctan2(vector_obstacle[1], vector_obstacle[0]) % (2 * np.pi)

        # print("robot_angle: {}, obstacle_angle: {}".format(degrees(robot_angle), degrees(obstacle_angle)))
        if j == len(obstacle_points) - 1 or (
                (robot_angle is not None) and (obstacle_angle is not None) and (robot_angle < obstacle_angle)):
            i += 1
        elif i == len(robot_points) - 1 or (
                (robot_angle is not None) and (obstacle_angle is not None) and robot_angle > obstacle_angle):
            j += 1
        else:
            i += 1
            j += 1

        if i >= len(robot_points) - 1 and j >= len(obstacle_points) - 1:
            break

    new_shape.append(tuple(map(lambda x_1, x_2: x_1 + x_2, robot_points[0], obstacle_points[0])))
    return Polygon(new_shape)


def get_combinations(item_list: typing.List[typing.Tuple]) -> typing.List[typing.Tuple]:
    """
    :param item_list: A list of items.
    :return: List of all pairs of distinct items from a list.
    """
    combinations = []
    for i in range(len(item_list)):
        for j in range(i + 1, len(item_list)):
            if not item_list[i][1] == item_list[j][1]:
                combinations.append((item_list[i][0], item_list[j][0]))
    return combinations


# TODO
def get_visibility_graph(obstacles: typing.List[Polygon], source=None, dest=None) -> typing.List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    vertices_and_polygon = []
    obstacles_edges = []
    for polygon in obstacles:
        obstacle_points = list(polygon.exterior.coords)
        for i in range(len(obstacle_points) - 1):
            vertices_and_polygon.append((obstacle_points[i], polygon))
            obstacles_edges.append(LineString([obstacle_points[i], obstacle_points[i + 1]]))
    if source is not None:
        vertices_and_polygon.append((source, None))
    if dest is not None:
        vertices_and_polygon.append((dest, None))
    combination_of_two_vertices = get_combinations(vertices_and_polygon)
    edges = obstacles_edges.copy()
    # edges = []
    for v1, v2 in combination_of_two_vertices:
        l = LineString([v1, v2])
        line_is_intersect_polygon = False
        for e in obstacles_edges:
            if e.coords[0] == v1 or e.coords[0] == v2 or e.coords[1] == v1 or e.coords[1] == v2:
                continue
            if l.intersects(LineString(e)):
                line_is_intersect_polygon = True
                break
        if line_is_intersect_polygon:
            continue
        d = l.length
        v3 = Point((v2[0] * (d + 1) - v1[0]) / d, (v2[1] * (d + 1) - v1[1]) / d)
        v4 = Point((v1[0] * (d + 1) - v2[0]) / d, (v1[1] * (d + 1) - v2[1]) / d)
        for obs in obstacles:
            if v3.intersection(obs) or v4.intersection(obs):
                line_is_intersect_polygon = True
                break
        if not line_is_intersect_polygon:
            edges.append(l)
    return edges


def dijkstra(graph: typing.Dict, start_vertex, goal_vertex=None) -> typing.Dict:
    """
    Computes the shortest path on a graph, given a starting vertex.
    :param graph: Dictionary: keys are vertices, items are dictionaries mapping neighbors (connected vertices) and
    their distance.
    :param start_vertex: An initial vertex to start the search from.
    :param goal_vertex: Optional goal vertex. If given, the search will be lazy and stop once the goal is found.
    Otherwise, will compute the shortest path to all vertices in the graph.
    :return: A dictionary mapping for each vertex in the graph a tuple of path cost, and predecessor vertex in the
    shortest path. The predecessor of source and non-examined vertices is None.
    """
    vertex_cost_and_predecessor = {v: (float('inf'), None) for v in graph.keys()}
    vertex_cost_and_predecessor[start_vertex] = (0, None)
    pq = PriorityQueue()
    pq.put((0, start_vertex))
    visited = []

    while not pq.empty():
        (dist, current_vertex) = pq.get()
        # If we're visiting the goal vertex, it means we just found the shortest path to it.
        if current_vertex == goal_vertex:
            break
        # Otherwise, keep searching by order of closest first.
        visited.append(current_vertex)
        neighbors = graph[current_vertex]
        for neighbor in neighbors:
            distance = neighbors[neighbor]
            if neighbor not in visited:
                old_cost = vertex_cost_and_predecessor[neighbor][0]
                new_cost = vertex_cost_and_predecessor[current_vertex][0] + distance
                if new_cost < old_cost:
                    pq.put((new_cost, neighbor))
                    vertex_cost_and_predecessor[neighbor] = (new_cost, current_vertex)
    return vertex_cost_and_predecessor


def get_shortest_path(lines: typing.List[LineString], source, dest) -> (typing.List, float):
    """
    Finds the shortest path from source point to destination point, given the C-space obstacles and the visibility
    graph.
    :param lines: List of LineString, each representing an edge in the visibility graph.
    :param source: Start point of the path. Should be a point of at least one of the lines.
    :param dest: End point of the path. Should be a point of at least one of the lines.
    :return:
    """
    # Construct a dictionary of the visibility graph, mapping for each vertex a dictionary of neighbors and their
    # distances.
    visibility_graph = dict()
    for e in lines:
        coords = mapping(e)['coordinates']
        for coord in coords:
            if coord not in visibility_graph:
                visibility_graph[coord] = dict()
        visibility_graph[coords[0]][coords[1]] = e.length
        visibility_graph[coords[1]][coords[0]] = e.length

    # Use dijkstra's algorithm to find the shortest path from source to dest.
    dijkstra_graph = dijkstra(visibility_graph, source, dest)
    shortest_path_vertices = list()
    path_cost = 0

    # The shortest path can be constructed from destination to source, tracking along the predecessors of the shortest
    # path until reaching the source. The predecessor of source will be None.
    shortest_path_vertices.append(dest)
    current_cost_and_vertex = dijkstra_graph[dest]
    while current_cost_and_vertex[1] is not None:
        shortest_path_vertices.append(current_cost_and_vertex[1])
        path_cost += current_cost_and_vertex[0]
        current_cost_and_vertex = dijkstra_graph[current_cost_and_vertex[1]]

    return shortest_path_vertices, path_cost


class VisibilityGraph:
    def __init__(self, edges: typing.List[LineString], obstacles, vertices=None):
        self.edges = edges
        if vertices is None:
            self.vertices = []
            for edge in edges:
                if edge.coords[0] not in self.vertices:
                    self.vertices.append(edge.coords[0])
                if edge.coords[1] not in self.vertices:
                    self.vertices.append(edge.coords[1])
        else:
            self.vertices = vertices
        # self.obstacles = obstacles
        self.obstacles_edges = []
        for polygon in obstacles:
            obstacle_points = list(polygon.exterior.coords)
            for i in range(len(obstacle_points) - 1):
                self.obstacles_edges.append(LineString([obstacle_points[i], obstacle_points[i + 1]]))
        self.goal = None
        self.start = None

    def get_combinations_to_target(self, target) -> typing.List[LineString]:
        """
        :param item_list: A list of items.
        :return: List of all pairs of distinct items from a list.
        """
        combinations = []
        for vertex in self.vertices:
            combinations.append((vertex, target))
        return combinations

    def set_goal(self, goal):
        for v1 in self.vertices:
            l = LineString([v1, goal])
            line_is_intersect_polygon = False
            for e in self.obstacles_edges:
                if e.coords[0] == v1 or e.coords[1] == v1:
                    continue
                if l.intersects(e):
                    # plt.figure()
                    # plt.plot(list(l.xy[0]), list(l.xy[1]), color='black', linewidth=1)
                    # plt.plot(list(e.xy[0]), list(e.xy[1]), color='blue', linewidth=1)
                    # plt.draw()  # draw the plot
                    # plt.pause(1) 115.0,-657.0
                    line_is_intersect_polygon = True
                    break
            # if line_is_intersect_polygon:
            #     continue
            # d = l.length
            # v3 = Point((goal[0] * (d + 1) - v1[0]) / d, (goal[1] * (d + 1) - v1[1]) / d)
            # v4 = Point((v1[0] * (d + 1) - goal[0]) / d, (v1[1] * (d + 1) - goal[1]) / d)
            # for obs in self.obstacles:
            #     if v3.intersection(obs) or v4.intersection(obs):
            #         line_is_intersect_polygon = True
            #         break
            if not line_is_intersect_polygon:
                self.edges.append(l)
        self.goal = goal
        if goal not in self.vertices:
            self.vertices.append(goal)

    def set_start(self, start):
        for v1 in self.vertices:
            l = LineString([start, v1])
            line_is_intersect_polygon = False
            for e in self.obstacles_edges:
                if e.coords[0] == v1 or e.coords[1] == v1:
                    continue
                if l.intersects(e):
                    line_is_intersect_polygon = True
                    break
            # if line_is_intersect_polygon:
            #     continue
            # d = l.length
            # v3 = Point((v1[0] * (d + 1) - start[0]) / d, (v1[1] * (d + 1) - start[1]) / d)
            # v4 = Point((start[0] * (d + 1) - v1[0]) / d, (start[1] * (d + 1) - v1[1]) / d)
            # for obs in self.obstacles:
            #     if v3.intersection(obs) or v4.intersection(obs):
            #         line_is_intersect_polygon = True
            #         break
            if not line_is_intersect_polygon:
                print
                self.edges.append(l)
        self.start = start
        if start not in self.vertices:
            self.vertices.append(start)

    def get_shortest_path(self):
        return get_shortest_path(self.edges, self.start, self.goal)

    def display_vertices(self):
        print("***** Vertices *****")
        for v in self.vertices:
            print(v)

    def display_edge(self):
        print("***** Edges *****")
        for e in self.edges:
            print(e.coords[0], e.coords[1])


if __name__ == '__main__':
    #parser = argparse.ArgumentParser()
    #parser.add_argument("Robot",
    #                    help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    #parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    #parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    #parser.add_argument("Vertices", help="A file that contains the vertices of the visibility graph.")
    #parser.add_argument("Edges", help="A file that contains the edges of the visibility graph.")
    #args = parser.parse_args()

    # obstacles = args.Obstacles
    # robot = args.Robot
    # query = args.Query
    # vertices = args.Vertices
    # edges = args.Edges

    obstacles = "./obstacles"
    robot = "./robot"
    query = "./query"
    vertices = "./vertices"
    edges = "./edges"
    lidar = "./points_file.txt"
    drone = "./drone_file.txt"

    #is_valid_file(parser, obstacles)
    #is_valid_file(parser, robot)
    #is_valid_file(parser, query)
    #is_valid_file(parser, vertices)
    #is_valid_file(parser, edges)

    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
           ob_vertices = line.split(' ')
           if ',' not in ob_vertices:
               ob_vertices = ob_vertices[:-1]
           points = [tuple(map(float, t.split(','))) for t in ob_vertices]
           workspace_obstacles.append(Polygon(points))

    c_space_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            c_space_obstacles.append(Polygon(points))

    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    c_space_vertices = []
    with open(vertices, 'r') as f:
        for line in f.readlines():
            vertices_line = line.split(' ')
            points = [tuple(map(float, t.split(','))) for t in vertices_line]
            c_space_vertices.append(points[0])

    c_space_edges = []
    with open(edges, 'r') as f:
        for line in f.readlines():
            edge_vertices = line.split(' ')
            points = [tuple(map(float, t.split(','))) for t in edge_vertices]
            c_space_edges.append(LineString(points))

    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    # step 1:
    #dist = 2
    # c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    # print("***** C_space_obstacles *****")
    # for obs in c_space_obstacles:
    #     print(list(obs.exterior.coords)[:-1])
    # plotter1 = Plotter()
    #
    # plotter1.add_obstacles(workspace_obstacles)
    # plotter1.add_c_space_obstacles(c_space_obstacles)
    # plotter1.add_robot(source, dist)

    # plotter1.show_graph()
    # print("Done 1")

    # step 2:
    #lines = get_visibility_graph(c_space_obstacles)
    #print(lines)
    # vg = VisibilityGraph(lines, c_space_obstacles)
    start_global_time = time.time()
    start_time = time.time()
    vg = VisibilityGraph(c_space_edges, c_space_obstacles, c_space_vertices)
    print("The time it took to build a visibility graph: {} sec".format(time.time() - start_time))
    # vg.display_vertices()
    # vg.display_edge()
    start_time = time.time()
    vg.set_goal(dest)
    print("The time it takes to enter a goal state: {} sec".format(time.time() - start_time))
    start_time = time.time()
    vg.set_start(source)
    print("The time it takes to enter a start state: {} sec".format(time.time() - start_time))
    start_time = time.time()
    shortest_path, _ = vg.get_shortest_path()
    print("The time it takes to find the shortest path: {} sec".format(time.time() - start_time))
    print("-----------------------------------------")
    print("Total time: {} sec".format(time.time() - start_global_time))




    lidar_points = []
    with open(lidar, 'r') as f:
        for line in f.readlines():
            vertices_line = line.split(' ')
            points = [tuple(map(float, t.split(','))) for t in vertices_line]
            lidar_points.append(points[0])

    drone_points = []
    with open(drone, 'r') as f:
        for line in f.readlines():
            vertices_line = line.split(' ')
            points = [tuple(map(float, t.split(','))) for t in vertices_line]
            drone_points.append(points[0])

    plotter4 = Plotter()
    plotter4.add_c_space_obstacles(c_space_obstacles)
    #plotter4.add_visibility_graph(vg.edges)
    #plotter4.add_robot(source, dist)
    #plotter4.add_shorterst_path(shortest_path)
    plotter4.add_lidar_points(lidar_points)
    plotter4.add_drone_points(drone_points)
    plotter4.show_graph()
    print("Done 4")

    print('Edges: {}'.format(len(vg.edges)))
    # for i,line in enumerate(lines):
    #     x, y = line.coords.xy
    #     print("edge num: {} - start_point=({},{}) end_point=({},{})".format(i+1, x[0], y[0], x[1], y[1]))
    # plotter2 = Plotter()
    #
    # plotter2.add_obstacles(workspace_obstacles)
    # plotter2.add_c_space_obstacles(c_space_obstacles)
    # plotter2.add_visibility_graph(lines)
    # plotter2.add_robot(source, dist)
    #
    # plotter2.show_graph()
    # print("Done 2")

    # step 3:
    # with open(query, 'r') as f:
    #     dest = tuple(map(float, f.readline().split(',')))
    #
    # lines = get_visibility_graph(c_space_obstacles, source, dest)
    # # TODO: fill in the next line
    # shortest_path, cost = get_shortest_path(lines, source, dest)
    # print(cost)
    # for e in shortest_path:
    #     print(e)
    #
    # plotter3 = Plotter()
    # plotter3.add_robot(source, dist)
    # plotter3.add_obstacles(workspace_obstacles)
    # plotter3.add_robot(dest, dist)
    # plotter3.add_visibility_graph(lines)
    # plotter3.add_shorterst_path(shortest_path)
    #
    # plotter3.show_graph()
    # print("Done 3")
