from ctypes import LibraryLoader
import airsim
import DroneTypes
import time
import numpy as np
from random import sample


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
        default_speed = 7
        threshold_angle = 30 * np.pi / 180
        self.flyToPosition(x, y, z, default_speed)

        time.sleep(1)

        while True:
            m_points = set()
            if len(self.getLidarData().points) > 1:
                print("Found obstacle...\n")

                # Find all points you can
                for i in range(25):
                    point = self.getLidarData().points
                    if len(point) > 1:
                        m_points.add(tuple(point))
                    time.sleep(0.04)
                print(str(len(m_points)) + " points were found!\n")

                left_point = sample(m_points, 1)[0]
                print(left_point)
                left_angle = np.arctan(left_point[0] / left_point[1])
                right_point = left_point
                right_angle = left_angle
                mid_angle = None
                mid_point = None
                for point in m_points:
                    current_angle = np.arctan(point[0] / point[1])
                    if current_angle < left_angle:
                        left_angle = current_angle
                        left_point = point
                    if current_angle > right_angle:
                        right_angle = current_angle
                        right_point = point
                    if np.abs(current_angle) < threshold_angle:
                        mid_angle = current_angle
                        mid_point = point
                
                print("Left point is - " + str(left_point))
                print("Right point is - " + str(right_point))
                print("mid point is - " + str(mid_point))
                print("mid angle is - " + str(mid_angle))

                # We can't continue straight
                if mid_angle is not None:
                    print("We cant continue straight")
                    current_pos = self.getPose().pos
                    current_orientation = self.getPose().orientation
                    
                    # Point positions in world frame
                    left_pos = [np.cos(-current_orientation.z_rad) * left_point[0] + np.sin(-current_orientation.z_rad) * left_point[1] + current_pos.x_m,
                                -np.sin(-current_orientation.z_rad) * left_point[0] + np.cos(-current_orientation.z_rad) * left_point[1] + current_pos.y_m,
                                current_pos.z_m]
                    right_pos = [np.cos(-current_orientation.z_rad) * right_point[0] + np.sin(-current_orientation.z_rad) * right_point[1] + current_pos.x_m,
                                -np.sin(-current_orientation.z_rad) * right_point[0] + np.cos(-current_orientation.z_rad) * right_point[1] + current_pos.y_m,
                                current_pos.z_m]

                    # dist = dist(point to drone) + dist(point to target)
                    left_dist = np.sqrt(left_point[0] ** 2 + left_point[1] ** 2) + np.sqrt((left_pos[0] - x) ** 2 + (left_pos[1] - y) ** 2)
                    right_dist = np.sqrt(right_point[0] ** 2 + right_point[1] ** 2) + np.sqrt((right_pos[0] - x) ** 2 + (right_pos[1] - y) ** 2)
                    if left_dist < right_dist:
                        destination = left_pos
                    else:
                        destination = right_pos

                    self.flyToPosition(destination[0], destination[1], destination[2], 10)
                
