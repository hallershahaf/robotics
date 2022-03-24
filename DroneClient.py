from ctypes import LibraryLoader
from importlib.machinery import DEBUG_BYTECODE_SUFFIXES
from re import S
from turtle import right
import airsim
import DroneTypes
import time
import numpy as np
from random import sample
from time import sleep

# Amount of points we scan every second
LIDAR_POINTS_NUM = 25
# In radians, any points with an angle lesser than this will be considered an obstacle
ANGLE_THRESHOLD = 7 * np.pi / 180
# In radians, the angle of the blindspot behind us where we ignore points
BLINDSPOT_ANGLE = 20 * np.pi / 180
# In radians, the angle in which we will follow obstacles without colliding
# NOT CURRENTLY USED
TRACING_ANGLE = 10 * np.pi / 180
# In meters, the distance in which we will follow obstacles without colliding
TRACING_DISTANCE = 5
# How close to the target do we have to be (in meters) final target
DISTANCE_THRESHOLD = 3
# How many meters to add to the points we are using to trace
DISTANCE_ADDED = 5
# Due to drone taking time to readjust, we let the code sleep for abit before continuing
READJUST_TIMEOUT = 1
# Speed we use for the drone, this might be removed later to use dynamic speeds
default_speed = 7

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
        self.flyToPosition(x, y, z, default_speed)
        sleep(READJUST_TIMEOUT)

        counter = 0
        was_tracing = False
        while self.isDroneAtPoint(x, y, z) is False:
            # If no points found then the function returns [0.0]
            if len(self.getLidarData().points) > 1:
                print("Found obstacle... ", end="")
                counter = 0

                # Find all points you can
                points_drone = []
                points_world = []
                for i in range(LIDAR_POINTS_NUM):
                    point = self.getLidarData().points
                    if len(point) > 1:
                        time.sleep(1 / LIDAR_POINTS_NUM)
                        points_drone += [point]
                        current_pose = self.getPose()
                        current_pos = current_pose.pos
                        orientation = current_pose.orientation
                        point_world = [np.cos(-orientation.z_rad) * point[0] + np.sin(-orientation.z_rad) * point[1] + current_pos.x_m,
                                        -np.sin(-orientation.z_rad) * point[0] + np.cos(-orientation.z_rad) * point[1] + current_pos.y_m,
                                        current_pos.z_m]
                        points_world += [point_world]

                left_point_index = 0
                drone_to_point = [points_world[left_point_index][0] - current_pos.x_m, points_world[left_point_index][1] - current_pos.y_m]
                left_angle = (np.arctan2(drone_to_point[1], drone_to_point[0]) - orientation.z_rad) + 2 * np.pi % (2 * np.pi)
                right_point_index = left_point_index
                right_angle = left_angle
                mid_point_index = None
                mid_angle = ANGLE_THRESHOLD

                # So we can decide if to trace right or left
                num_of_left_angles = 0
                num_of_right_angles = 0

                drone_to_target = [x - current_pos.x_m, y - current_pos.y_m]
                drone_to_target_angle = np.arctan2(drone_to_target[1], drone_to_target[0]) - orientation.z_rad + 2 * np.pi % (2 * np.pi)

                # We are looking for the farthest points we have to the left and right, and if there are points right in front of us
                for index, point in enumerate(points_world):
                    drone_to_point = [points_world[index][0] - current_pos.x_m, points_world[index][1] - current_pos.y_m]
                    current_angle_unnormal = np.arctan2(drone_to_point[1], drone_to_point[0]) - orientation.z_rad
                    # Normalize angle to [-pi, pi]
                    current_angle = current_angle_unnormal + 2 * np.pi % (2 * np.pi)

                    # Points straight behind us cause a problem in the transition between -pi to pi
                    # and frankly aren't that relevant anyway, so we can ignore them
                    if np.abs(current_angle) < (np.pi - BLINDSPOT_ANGLE):
                        print(current_angle)
                        # Count angles to determine which way to trace
                        if current_angle < 0:
                            num_of_right_angles += 1
                        else:
                            num_of_left_angles += 1
                        if current_angle < right_angle:
                            right_angle = current_angle
                            right_point_index = index
                        if current_angle > left_angle:
                            left_angle = current_angle
                            left_point_index = index
                        if np.abs(current_angle - drone_to_target_angle) < np.abs(mid_angle):
                            mid_angle = current_angle
                            mid_point_index = index

                print("*****")
                print("right angle is")
                print(right_angle)
                print("left angle is")
                print(left_angle)

                # Obstacle is blocking, tracing it until we have a clear vision to the target again
                if mid_point_index is None:
                    if was_tracing:
                        was_tracing = False
                        print("Done tracing")
                        self.flyToPosition(x, y, z, default_speed)
                    else:
                        print("Obstacle not in the way, ignoring\n")
                else:
                    current_dist = np.sqrt((current_pos.x_m - x) ** 2 + (current_pos.y_m - y) ** 2)
                    mid_point_dist = np.sqrt(points_drone[mid_point_index][0] ** 2 + points_drone[mid_point_index][1] ** 2)

                    # If the target is closer than the points we found, we ignore them.
                    if current_dist < mid_point_dist:
                        print("Obstacle is further than target, ignoring\n")
                    else:
                        print("Obstacle is blocking, tracing\n")
                        was_tracing = True
                        if num_of_left_angles > num_of_right_angles: # Tracing right
                            # Change points from world perspective to drone perspective (original list might be irrelevant due to drone's movement)
                            right_point_drone = [np.cos(orientation.z_rad) * (points_world[right_point_index][0] - current_pos.x_m) + np.sin(orientation.z_rad) * (points_world[right_point_index][1] - current_pos.y_m),
                                                -np.sin(orientation.z_rad) * (points_world[right_point_index][0] - current_pos.x_m) + np.cos(orientation.z_rad) * (points_world[right_point_index][1] - current_pos.y_m),
                                                current_pos.z_m]

                            # We increase the size of the points so we will actually travel past them (after adjusting angles), so we will pass obstacles and not get stuck at the edge.
                            right_point_size = np.sqrt(right_point_drone[0] ** 2 + right_point_drone[1] ** 2)
                            right_point_bigger = [right_point_drone[0] / right_point_size * (right_point_size + DISTANCE_ADDED), right_point_drone[1] / right_point_size * (right_point_size + DISTANCE_ADDED)]

                            # Find the angle to move the point so we will be TRACING_DISTANCE meters apart
                            # Math is derived from cosine theorem
                            #print(str((2 * (right_point_size ** 2)) - (TRACING_DISTANCE ** 2)) / (2 * (right_point_size ** 2)))
                            tracing_angle = np.abs(np.arccos(((2 * (right_point_size ** 2)) - (TRACING_DISTANCE ** 2)) / (2 * (right_point_size ** 2))))

                            # Now we adjust the angles so we will stay away from the obstacles
                            right_point_moved = [np.cos(tracing_angle) * right_point_bigger[0] + np.sin(tracing_angle) * right_point_bigger[1],
                                            -np.sin(tracing_angle) * right_point_bigger[0] + np.cos(tracing_angle) * right_point_bigger[1]]

                            # Return to world perspective so we know where to fly
                            right_point_world = [np.cos(-orientation.z_rad) * right_point_moved[0] + np.sin(-orientation.z_rad) * right_point_moved[1] + current_pos.x_m,
                                                -np.sin(-orientation.z_rad) * right_point_moved[0] + np.cos(-orientation.z_rad) * right_point_moved[1] + current_pos.y_m,
                                                current_pos.z_m]

                            self.flyToPosition(right_point_world[0], right_point_world[1], current_pos.z_m, default_speed)
                            sleep(READJUST_TIMEOUT)
                        else: # Tracing left
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

                            self.flyToPosition(left_point_world[0], left_point_world[1], current_pos.z_m, default_speed)
                            sleep(READJUST_TIMEOUT)
            # if len(self.getLidarData().points) > 1:
            else:
                # If we don't find anything within LIDAR_POINTS_NUM points (full scan)
                counter += 1
                if counter == LIDAR_POINTS_NUM * 2 and was_tracing:
                    counter = 0
                    was_tracing = False
                    self.flyToPosition(x, y, z, default_speed)
                time.sleep(1 / LIDAR_POINTS_NUM)

        print("Reached ({},{},{})\n".format(x, y, z))

    def isDroneAtPoint(self, x, y, z):
        current_pos = self.getPose().pos
        current_dist = np.sqrt((current_pos.x_m - x) ** 2 + (current_pos.y_m - y) ** 2 + (current_pos.z_m - z) ** 2)
        if current_dist < DISTANCE_THRESHOLD:
            return True
        else:
            return False
