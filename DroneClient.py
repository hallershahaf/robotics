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

        while True:
            # Default/First state : continue straight until obstacle is blocking
            print("First state - continuing straight\n")
            self.flyToPosition(x, y, z, default_speed)

            # If no points found then the function returns [0.0]
            if len(self.getLidarData().points) > 1:
                print("Found obstacle...\n")

                # Find all points you can
                left, right, mid, num_of_angles = self.getLidarPointsInfo()                
                print("Left point is - " + str(left[0]))
                print("Right point is - " + str(right[0]))
                print("mid point is - " + str(mid[0]))
                print("mid angle is - " + str(mid[1]))

                # Second state : Obstacle is blocking, go to best heuristic distance
                if mid[0] is not None:
                    print("Second state - we cant continue straight\n")
                    current_pos = self.getPose().pos
                    current_orientation = self.getPose().orientation
                    
                    # Point positions in world frame
                    left_pos = [np.cos(-current_orientation.z_rad) * left[0][0] + np.sin(-current_orientation.z_rad) * left[0][1] + current_pos.x_m,
                                -np.sin(-current_orientation.z_rad) * left[0][0] + np.cos(-current_orientation.z_rad) * left[0][1] + current_pos.y_m,
                                current_pos.z_m]
                    right_pos = [np.cos(-current_orientation.z_rad) * right[0][0] + np.sin(-current_orientation.z_rad) * right[0][1] + current_pos.x_m,
                                -np.sin(-current_orientation.z_rad) * right[0][0] + np.cos(-current_orientation.z_rad) * right[0][1] + current_pos.y_m,
                                current_pos.z_m]

                    # dist = dist(point to drone) + dist(point to target)
                    current_dist = np.sqrt((current_pos.x_m - x) ** 2 + (current_pos.y_m - y) ** 2)
                    left_dist = np.sqrt(left[0][0] ** 2 + left[0][1] ** 2) + np.sqrt((left_pos[0] - x) ** 2 + (left_pos[1] - y) ** 2)
                    right_dist = np.sqrt(right[0][0] ** 2 + right[0][1] ** 2) + np.sqrt((right_pos[0] - x) ** 2 + (right_pos[1] - y) ** 2)
                    # As long as we can improve our position
                    if current_dist > left_dist and current_dist > right_dist:
                        if left_dist < right_dist:
                            destination = left_pos
                        else:
                            destination = right_pos
                        # TODO: improve exact location to move to
                        self.flyToPosition(destination[0], destination[1], destination[2], 10)

                    # Third state : can't improve distance, tracing obstacle
                    else:
                        print("Third state - tracing object\n")
                        # TODO: make it stop, not just trace forever
                        while True:
                            left, right, mid = self.getLidarPointsInfo()
                            # Trace to the right
                            if num_of_angles[0] > num_of_angles[1]:
                                print("Tracing to the right\n")
                                self.flyToPosition(right[0][0], right[0][1], right[0][2], 10)
                            # Trace to the left
                            else:
                                print("Tracing to the left\n")
                                self.flyToPosition(left[0][0], left[0][1], left[0][2], 10)

    def getLidarPointsInfo(self):
        threshold_angle = 30 * np.pi / 180
        m_points = set()

        # Find all points you can
        for i in range(25):
            point = self.getLidarData().points
            if len(point) > 1:
                m_points.add(tuple(point))
            time.sleep(0.04)
        print(str(len(m_points)) + " points were found!\n")

        left_point = sample(m_points, 1)[0]
        print(left_point)
        left_angle = np.arctan(left_point[1] / left_point[0])
        right_point = left_point
        right_angle = left_angle
        mid_angle = None
        mid_point = None

        # So we can decide if to trace right or left
        num_of_left_angles = 0
        num_of_right_angles = 0

        # Left side has negative y values, and therefore negative angles
        for point in m_points:
            current_angle = np.arctan(point[1] / point[0])
            if current_angle < 0:
                num_of_left_angles += 1
            else:
                num_of_right_angles += 1
            if current_angle < left_angle:
                left_angle = current_angle
                left_point = point
            if current_angle > right_angle:
                right_angle = current_angle
                right_point = point
            if np.abs(current_angle) < threshold_angle:
                mid_angle = current_angle
                mid_point = point

        return [left_point, left_angle], [right_point, right_angle], [mid_point, mid_angle], [num_of_left_angles, num_of_right_angles]
