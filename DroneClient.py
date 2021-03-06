from ctypes import LibraryLoader
from importlib.machinery import DEBUG_BYTECODE_SUFFIXES
from re import S
from turtle import right
import airsim
import DroneTypes
import time
import numpy as np
from time import sleep
# This library plots the direction
from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString, Point
from shapely.geometry import mapping
from visibility_graph_and_shortest_path import get_points_and_dist
from visibility_graph_and_shortest_path import VisibilityGraph as VG

# Amount of points we scan every second
LIDAR_POINTS_NUM = 25
DELAY_BETWEEN_POINTS = 1 / 25
# Angle cone in front of the drone where it ignores points outside of it when tracing
ANGLE_BLINDSPOT = 25 * np.pi / 180
# In radians, any points with an angle lesser than this will be considered an obstacle
ANGLE_THRESHOLD = 12 * np.pi / 180
# In radians, the angle of the blindspot behind us where we ignore points
BEHIND_BLINDSPOT_ANGLE = 20 * np.pi / 180
# In meters, the distance in which we will follow obstacles without colliding
TRACING_DISTANCE = 5
# How close to the target do we have to be (in meters) final target
DISTANCE_THRESHOLD = 3
# Distance from dikstra vertice we ignore
DISTANCE_TO_VERTICE_THRESHOLD = 5
# How many meters to add to the points we are using to trace
DISTANCE_ADDED = 15
# Due to drone taking time to readjust, we let the code sleep for abit before continuing
READJUST_TIMEOUT = 2.0
# Minimum amount of time required for the drone to complete a command
COMMAND_TIME = 0.8
# Speed we use for the drone, this might be removed later to use dynamic speeds
straight_default_speed = 4
left_default_speed = 3
right_default_speed = 2

# direction states
RIGHT = 1
LEFT = -1
NONE = 0

class DroneClient:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.future = None

    def __del__(self):
        if self.future is not None:
            self.future.join()

    def connect(self):
        """
        Connect to simulation

        Args:
            none

        Returns:
            none
        """
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def isConnected(self):
        """
        Check if client is connected to simulation

        Args:
            none

        Returns:
            bool: true if connected. otherwise return false
        """
        return self.client.isApiControlEnabled()

    def getPose(self):
        """
        Get the pose of the drone

        Args:
            none

        Returns:
            DroneTypes.Pose : the pose of the drone
        """
        drone_pose = self.client.simGetVehiclePose()
        res = DroneTypes.Pose()

        res.pos.x_m = drone_pose.position.x_val
        res.pos.y_m = drone_pose.position.y_val
        res.pos.z_m = drone_pose.position.z_val

        euler = airsim.utils.to_eularian_angles(drone_pose.orientation)

        res.orientation.x_rad = euler[0]
        res.orientation.y_rad = euler[1]
        res.orientation.z_rad = euler[2]

        return res

    def getLidarData(self):
        point_cloud = DroneTypes.PointCloud()
        lidar_data = self.client.getLidarData()

        point_cloud.points = lidar_data.point_cloud

        return point_cloud

    def flyToPosition(self, x: float, y: float, z: float, v: float):
        """
        Fly the drone to position

        Args:
            x : float - x coordinate
            y : float - y coordinate
            z : float - z coordinate
            v : float - the velocity which the drone fly to position

        Returns:
            none
        """
        self.future = self.client.moveToPositionAsync(x, y, z, v, drivetrain=airsim.DrivetrainType.ForwardOnly,
                                                      yaw_mode=airsim.YawMode(False, 0.0))

    def setAtPosition(self, x: float, y: float, z: float):
        """
        Set the drone at position instantly

        Args:
            x : float - x coordinate
            y : float - y coordinate
            z : float - z coordinate

        Returns:
            none
        """
        pos = airsim.Vector3r(x, y, z)
        q = airsim.Quaternionr(1, 0, 0, 0)
        pose = airsim.Pose(pos, q)

        self.client.simSetVehiclePose(pose, True)
        self.flyToPosition(x, y, z, 1)

    def reset(self):
        """
        Returns the drone to start position

        Args:
            none

        Returns:
            none
        """
        self.client.reset()

    def goTo(self, x: float, y: float, z: float):
        """
        Uses flyToPosition and motion planning algorithms to get to (x,y,z) without collision.

        Args:
            x : float - x coordinate
            y : float - y coordinate
            z : float - z coordinate

        Returns:
            none
        """
        obstacles = "./c_space_obstacles2"
        robot = "./robot"
        query = "./query"
        vertices = "./vertices2"
        edges = "./edges2"

        file_points = open("D:\Technion\Intro_Robotics\project\points_file.txt", "w")
        file_drone = open("D:\Technion\Intro_Robotics\project\drone_file.txt", "w")

        c_space_obstacles = []
        with open(obstacles, 'r') as f:
            for line in f.readlines():
                ob_vertices = line.split(' ')
                if ',' not in ob_vertices:
                    ob_vertices = ob_vertices[:-1]
                points = [tuple(map(float, t.split(','))) for t in ob_vertices]
                c_space_obstacles.append(Polygon(points))

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

        pos = self.getPose().pos
        source = (x, y)
        dest = (pos.x_m, pos.y_m)
        vg = VG(c_space_edges, c_space_obstacles, c_space_vertices)
        vg.set_start(source)
        vg.set_goal(dest)
        path = vg.get_shortest_path()
        
        dist_to_path = np.sqrt((path[0][1][0] - pos.x_m) ** 2 + (path[0][1][1] - pos.y_m) ** 2)
        speed_to_path = dist_to_path / COMMAND_TIME
        vertice = 1
        if dist_to_path < DISTANCE_TO_VERTICE_THRESHOLD:
            vertice = 2
        self.flyToPosition(path[0][vertice][0], path[0][vertice][1], z, np.minimum(straight_default_speed, speed_to_path))
        sleep(READJUST_TIMEOUT)

        counter = 0
        criteria_counter = 0
        was_tracing = False
        trace_direction = None
        while self.isDroneAtPoint(x, y, z) is False:
            current_pose = self.getPose()
            current_pos = current_pose.pos
            file_drone.write(str(current_pos.x_m) + "," + str(current_pos.y_m) + "\n")
            file_drone.flush()
            point = self.getLidarData().points
            if len(point) > 1:
                current_pose = self.getPose()
                current_pos = current_pose.pos
                orientation = current_pose.orientation
                point_world = [np.cos(-orientation.z_rad) * point[0] + np.sin(-orientation.z_rad) * point[1] + current_pos.x_m,
                                -np.sin(-orientation.z_rad) * point[0] + np.cos(-orientation.z_rad) * point[1] + current_pos.y_m,
                                current_pos.z_m]
                file_points.write(str(point_world[0]) + "," + str(point_world[1]) + "\n")
                file_points.flush()
            # If no points found then the function returns [0.0]
            # Ignore points if we are not tracing, and they are far away
            if len(point) > 1 and (was_tracing or (np.abs(np.arctan2(point[1], point[0])) < ANGLE_BLINDSPOT)):
                counter = 0
                points_drone = []
                points_world = []
                if not was_tracing:
                    print("Found obstacle... ", end="")
                    # Find all points you can
                    for i in range(LIDAR_POINTS_NUM):
                        point = self.getLidarData().points
                        if len(point) > 1:
                            time.sleep(DELAY_BETWEEN_POINTS)
                            points_drone += [point]
                            current_pose = self.getPose()
                            current_pos = current_pose.pos
                            orientation = current_pose.orientation
                            point_world = [np.cos(-orientation.z_rad) * point[0] + np.sin(-orientation.z_rad) * point[1] + current_pos.x_m,
                                            -np.sin(-orientation.z_rad) * point[0] + np.cos(-orientation.z_rad) * point[1] + current_pos.y_m,
                                            current_pos.z_m]
                            points_world += [point_world]
                else:
                    for i in range(LIDAR_POINTS_NUM):
                        pose = self.getPose()
                        orientation = pose.orientation
                        current_pos = pose.pos
                        drone_to_target = [x - current_pos.x_m, y - current_pos.y_m]
                        drone_to_target_angle = np.arctan2(drone_to_target[1], drone_to_target[0]) - orientation.z_rad

                        point = self.getLidarData().points
                        if len(point) > 1 and (np.abs(drone_to_target_angle - np.arctan2(point[1], point[0])) < (np.pi / 2)
                                               or np.abs(np.arctan2(point[1], point[0]) < (ANGLE_BLINDSPOT * 3 / 5))):
                            print(point)
                            time.sleep(DELAY_BETWEEN_POINTS)
                            points_drone += [point]
                            current_pose = self.getPose()
                            current_pos = current_pose.pos
                            orientation = current_pose.orientation
                            point_world = [np.cos(-orientation.z_rad) * point[0] + np.sin(-orientation.z_rad) * point[1] + current_pos.x_m,
                                            -np.sin(-orientation.z_rad) * point[0] + np.cos(-orientation.z_rad) * point[1] + current_pos.y_m,
                                            current_pos.z_m]
                            points_world += [point_world]

                if len(points_drone) == 0:
                    print("no points entered criteria")
                    criteria_counter += 1
                    if criteria_counter == 4:
                        criteria_counter = 0
                        was_tracing = False
                        trace_direction = None
                        pos = self.getPose().pos
                        dest = (pos.x_m, pos.y_m)
                        vg.set_goal(dest)
                        path = vg.get_shortest_path()
                        dist_to_path = np.sqrt((path[0][1][0] - pos.x_m) ** 2 + (path[0][1][1] - pos.y_m) ** 2)
                        speed_to_path = dist_to_path / COMMAND_TIME
                        self.flyToPosition(path[0][1][0], path[0][1][1], z, np.minimum(straight_default_speed, speed_to_path))
                    sleep(READJUST_TIMEOUT)
                    continue
                left_point_index = 0
                drone_to_point = [points_world[left_point_index][0] - current_pos.x_m, points_world[left_point_index][1] - current_pos.y_m]
                left_angle = np.arctan2(drone_to_point[1], drone_to_point[0]) - orientation.z_rad
                right_point_index = left_point_index
                right_angle = left_angle
                mid_point_index = None
                mid_angle = ANGLE_THRESHOLD

                drone_to_target = [x - current_pos.x_m, y - current_pos.y_m]
                drone_to_target_angle = np.arctan2(drone_to_target[1], drone_to_target[0]) - orientation.z_rad

                # We are looking for the farthest points we have to the left and right, and if there are points right in front of us
                for index, point in enumerate(points_world):
                    orientation = self.getPose().orientation
                    drone_to_point = [points_world[index][0] - current_pos.x_m, points_world[index][1] - current_pos.y_m]
                    current_angle = np.arctan2(drone_to_point[1], drone_to_point[0]) - orientation.z_rad
                    if current_angle < -np.pi:
                        current_angle = current_angle % np.pi

                    # Points straight behind us cause a problem in the transition between -pi to pi
                    # and frankly aren't that relevant anyway, so we can ignore them
                    if np.abs(current_angle) < (np.pi - BEHIND_BLINDSPOT_ANGLE):
                        # Count angles to determine which way to trace
                        if current_angle < right_angle:
                            right_angle = current_angle
                            right_point_index = index
                        if current_angle > left_angle:
                            left_angle = current_angle
                            left_point_index = index
                        if np.abs(current_angle - drone_to_target_angle) < np.abs(mid_angle):
                            mid_angle = current_angle
                            mid_point_index = index

                # Obstacle is blocking, tracing it until we have a clear vision to the target again
                if  (trace_direction == LEFT and drone_to_target_angle - left_angle > ANGLE_THRESHOLD * 3) or \
                    (trace_direction == RIGHT and drone_to_target_angle - right_angle < ANGLE_THRESHOLD * 3):
                        was_tracing = False
                        trace_direction = None
                        print("Done tracing")
                        pos = self.getPose().pos
                        dest = (pos.x_m, pos.y_m)
                        vg.set_goal(dest)
                        path = vg.get_shortest_path()
                        dist_to_path = np.sqrt((path[0][1][0] - pos.x_m) ** 2 + (path[0][1][1] - pos.y_m) ** 2)
                        speed_to_path = dist_to_path / COMMAND_TIME
                        vertice = 1
                        if dist_to_path < DISTANCE_TO_VERTICE_THRESHOLD:
                            vertice = 2
                        self.flyToPosition(path[0][vertice][0], path[0][vertice][1], z, np.minimum(straight_default_speed, speed_to_path))
                        sleep(READJUST_TIMEOUT)
                elif was_tracing or mid_point_index is not None:
                    current_dist = np.sqrt((current_pos.x_m - x) ** 2 + (current_pos.y_m - y) ** 2)
                    if mid_point_index is not None:
                        mid_point_dist = np.sqrt(points_drone[mid_point_index][0] ** 2 + points_drone[mid_point_index][1] ** 2)

                    # If the target is closer than the points we found, we ignore them.
                    if mid_point_index is not None and current_dist < mid_point_dist:
                        print("Obstacle is further than target, ignoring\n")
                    else:
                        print("Obstacle is blocking, tracing\n")
                        was_tracing = True
                        dist_right_point = np.sqrt(((points_world[right_point_index][0] - x)**2 +(points_world[right_point_index][1] - y)**2))
                        dist_left_point = np.sqrt(((points_world[left_point_index][0] - x)**2 +(points_world[left_point_index][1] - y)**2))
                        if dist_left_point > dist_right_point and trace_direction != LEFT: # Tracing right
                            trace_direction = RIGHT
                            # Change points from world perspective to drone perspective (original list might be irrelevant due to drone's movement)
                            right_point_drone = [np.cos(orientation.z_rad) * (points_world[right_point_index][0] - current_pos.x_m) + np.sin(orientation.z_rad) * (points_world[right_point_index][1] - current_pos.y_m),
                                                -np.sin(orientation.z_rad) * (points_world[right_point_index][0] - current_pos.x_m) + np.cos(orientation.z_rad) * (points_world[right_point_index][1] - current_pos.y_m),
                                                current_pos.z_m]

                            # We increase the size of the points so we will actually travel past them (after adjusting angles), so we will pass obstacles and not get stuck at the edge.
                            right_point_size = np.sqrt(right_point_drone[0] ** 2 + right_point_drone[1] ** 2)
                            right_point_bigger = [right_point_drone[0] / right_point_size * (right_point_size + DISTANCE_ADDED), right_point_drone[1] / right_point_size * (right_point_size + DISTANCE_ADDED)]

                            # Find the angle to move the point so we will be TRACING_DISTANCE meters apart
                            # Math is derived from cosine theorem
                            tracing_angle = np.abs(np.arccos(((2 * (right_point_size ** 2)) - (TRACING_DISTANCE ** 2)) / (2 * (right_point_size ** 2))))

                            # Now we adjust the angles so we will stay away from the obstacles
                            right_point_moved = [np.cos(tracing_angle) * right_point_bigger[0] + np.sin(tracing_angle) * right_point_bigger[1],
                                            -np.sin(tracing_angle) * right_point_bigger[0] + np.cos(tracing_angle) * right_point_bigger[1]]

                            # Return to world perspective so we know where to fly
                            right_point_world = [np.cos(-orientation.z_rad) * right_point_moved[0] + np.sin(-orientation.z_rad) * right_point_moved[1] + current_pos.x_m,
                                                -np.sin(-orientation.z_rad) * right_point_moved[0] + np.cos(-orientation.z_rad) * right_point_moved[1] + current_pos.y_m,
                                                current_pos.z_m]

                            pos = self.getPose().pos
                            dist_to_path = np.sqrt((path[0][1][0] - pos.x_m) ** 2 + (path[0][1][1] - pos.y_m) ** 2)
                            speed_to_path = dist_to_path / COMMAND_TIME
                            self.flyToPosition(right_point_world[0], right_point_world[1], z, np.minimum(right_default_speed, speed_to_path))
                            sleep(READJUST_TIMEOUT)
                        else: # Tracing left
                            trace_direction = LEFT
                            # Change points from world perspective to drone perspective (original list might be irrelevant due to drone's movement)
                            left_point_drone = [np.cos(orientation.z_rad) * (points_world[left_point_index][0] - current_pos.x_m) + np.sin(orientation.z_rad) * (points_world[left_point_index][1] - current_pos.y_m),
                                                -np.sin(orientation.z_rad) * (points_world[left_point_index][0] - current_pos.x_m) + np.cos(orientation.z_rad) * (points_world[left_point_index][1] - current_pos.y_m),
                                                current_pos.z_m]

                            # We increase the size of the points so we will actually travel past them (after adjusting angles), so we will pass obstacles and not get stuck at the edge.
                            left_point_size = np.sqrt(left_point_drone[0] ** 2 + left_point_drone[1] ** 2)
                            left_point_bigger = [left_point_drone[0] / left_point_size * (left_point_size + DISTANCE_ADDED), left_point_drone[1] / left_point_size * (left_point_size + DISTANCE_ADDED)]

                            # Find the angle to move the point so we will be TRACING_DISTANCE meters apart from the object
                            # Math is derived from cosine theorem
                            tracing_angle = np.abs(np.arccos(((2 * (left_point_size ** 2)) - (TRACING_DISTANCE ** 2)) / (2 * (left_point_size ** 2))))

                            # Now we adjust the angles so we will stay away from the obstacles
                            left_point_moved = [np.cos(-tracing_angle) * left_point_bigger[0] + np.sin(-tracing_angle) * left_point_bigger[1],
                                            -np.sin(-tracing_angle) * left_point_bigger[0] + np.cos(-tracing_angle) * left_point_bigger[1]]

                            # Return to world perspective so we know where to fly
                            left_point_world = [np.cos(-orientation.z_rad) * left_point_moved[0] + np.sin(-orientation.z_rad) * left_point_moved[1] + current_pos.x_m,
                                                -np.sin(-orientation.z_rad) * left_point_moved[0] + np.cos(-orientation.z_rad) * left_point_moved[1] + current_pos.y_m,
                                                current_pos.z_m]

                            pos = self.getPose().pos
                            dist_to_path = np.sqrt((path[0][1][0] - pos.x_m) ** 2 + (path[0][1][1] - pos.y_m) ** 2)
                            speed_to_path = dist_to_path / COMMAND_TIME
                            self.flyToPosition(left_point_world[0], left_point_world[1], z, np.minimum(left_default_speed, speed_to_path))
            else:
                # If we don't find anything within 2 LIDAR_POINTS_NUM points (double scan)
                counter += 1
                if counter == LIDAR_POINTS_NUM * 2:
                    counter = 0
                    was_tracing = False
                    trace_direction = None
                    pos = self.getPose().pos
                    dest = (pos.x_m, pos.y_m)
                    vg.set_goal(dest)
                    path = vg.get_shortest_path()

                    pos = self.getPose().pos
                    dist_to_path = np.sqrt((path[0][1][0] - pos.x_m) ** 2 + (path[0][1][1] - pos.y_m) ** 2)
                    speed_to_path = dist_to_path / COMMAND_TIME
                    vertice = 1
                    if dist_to_path < DISTANCE_TO_VERTICE_THRESHOLD:
                        vertice = 2
                    self.flyToPosition(path[0][vertice][0], path[0][vertice][1], z, np.minimum(straight_default_speed, speed_to_path))
                    sleep(READJUST_TIMEOUT)
                time.sleep(DELAY_BETWEEN_POINTS)

        print("Reached ({},{},{})\n".format(x, y, z))

    def isDroneAtPoint(self, x, y, z):
        current_pos = self.getPose().pos
        current_dist = np.sqrt((current_pos.x_m - x) ** 2 + (current_pos.y_m - y) ** 2 + (current_pos.z_m - z) ** 2)
        if current_dist < DISTANCE_THRESHOLD:
            return True
        else:
            return False
