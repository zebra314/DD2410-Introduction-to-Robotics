#!/usr/bin/env python3

"""
    # Ying Pei Lin
    # {student id}
    # yplin@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs, sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap

class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        yaw_robot = self.get_yaw(pose.pose.orientation)

        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()

        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        """
        Fill in your solution here
        """        

        # Calculate the robot's coordinates relative to the map
        x_robot = pose.pose.position.x - origin.position.x
        y_robot = pose.pose.position.y - origin.position.y

        # Record the min and max of the updated map
        # Used for the C part
        x_max, y_max = 0, 0
        x_min, y_min = grid_map.get_width(), grid_map.get_height()            

        obstacles = []

        for i in range(len(scan.ranges)):

            # Skip invalid measurements
            if scan.ranges[i] <= scan.range_min or scan.ranges[i] >= scan.range_max:
                continue

            # Calculate the bearing of the laser scan
            bearing = scan.angle_min + i * scan.angle_increment

            # Calculate the obstacle coordinates relative to the map
            x_obs = scan.ranges[i] * cos(bearing + yaw_robot ) + x_robot
            y_obs = scan.ranges[i] * sin(bearing + yaw_robot ) + y_robot

            # Convert the obstacle coordinates to map indices
            x_obs = int(x_obs / resolution)
            y_obs = int(y_obs / resolution)

            # Record the obstacle coordinates
            obstacles.append((x_obs, y_obs))

            # Set the space between the robot and the obstacle as free space
            traversed_cells = self.raytrace([int(x_robot / resolution), int(y_robot / resolution)], [x_obs, y_obs])
            for traversed_cell in traversed_cells:
                (x_traversed, y_traversed) = traversed_cell
                self.add_to_map(grid_map, x_traversed, y_traversed, self.free_space)     

        # Set the obstacle space
        for obstacle in obstacles:
            (x_obs, y_obs) = obstacle
            x_max, x_min = max(x_max, x_obs), min(x_min, x_obs)
            y_max, y_min = max(y_max, y_obs), min(y_min, y_obs)
            self.add_to_map(grid_map, x_obs, y_obs, self.occupied_space)

        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = x_min
        # The minimum y index in 'grid_map' that has been updated
        update.y = y_min
        # Maximum x index - minimum x index + 1
        update.width = x_max - x_min + 1
        # Maximum y index - minimum y index + 1
        update.height = y_max - y_min + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        for w in range(update.width):
            for h in range(update.height):
                update.data.append(grid_map[update.x + w, update.y + h])

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def expansion(self, point):
        # Extend a point to an area with radius of self.radius
        point_x = point[0]
        point_y = point[1]
        expanded_points = []
        expanded_points.append([point_x + 4, point_y])
        expanded_points.append([point_x - 4, point_y])
        expanded_points.append([point_x, point_y + 4])
        expanded_points.append([point_x, point_y - 4])

        for i in range(4):
            for j in range(4):
                expanded_points.append([point_x + i, point_y + j])
                expanded_points.append([point_x - i, point_y - j])
                expanded_points.append([point_x + i, point_y - j])
                expanded_points.append([point_x - i, point_y + j])

        expanded_points.remove([point_x + 3, point_y + 3])
        expanded_points.remove([point_x + 3, point_y - 3])
        expanded_points.remove([point_x - 3, point_y + 3])
        expanded_points.remove([point_x - 3, point_y - 3])

        return expanded_points

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        widths = grid_map.get_width()
        heights = grid_map.get_height()
        origin = grid_map.get_origin()

        # Loop through the grid map
        for w in range(widths):
            for h in range(heights):

                # Calculate the index of the cell
                cell_x = int(origin.position.x + w)
                cell_y = int(origin.position.y + h)
                
                # Skip if the cell is not occupied or out of bounds
                if not self.is_in_bounds(grid_map, cell_x, cell_y):
                    continue
                if not grid_map[cell_x, cell_y] == self.occupied_space:
                    continue

                # Inflate the cell
                expended_cells = self.expansion([cell_x, cell_y])
                for expended_cell in expended_cells:

                    # If the cell is already occupied, skip
                    if grid_map[expended_cell[0], expended_cell[1]] == self.occupied_space:
                        continue    
                    self.add_to_map(grid_map, expended_cell[0], expended_cell[1], self.c_space)

        # Return the inflated map
        return grid_map
