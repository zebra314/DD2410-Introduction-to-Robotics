#!/usr/bin/env python3

"""
    # {Rui Qu}
    # {19930307-T550}
    # {rqu@kth.se}
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
        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        map_origin = grid_map.get_origin()
        # The map resolution [m/cell]
        map_resolution = grid_map.get_resolution()

        obstacles = []
        obstacle_x_coords = []
        obstacle_y_coords = []

        robot_x = pose.pose.position.x - map_origin.position.x
        robot_y = pose.pose.position.y - map_origin.position.y

        for i in range(len(scan.ranges)):
            if scan.ranges[i] > scan.range_min and scan.ranges[i] < scan.range_max:
                bearing_angle = scan.angle_min + i * scan.angle_increment
                obstacle_x = scan.ranges[i] * cos(bearing_angle + robot_yaw) + robot_x
                obstacle_y = scan.ranges[i] * sin(bearing_angle + robot_yaw) + robot_y

                obstacle_x = int(obstacle_x / map_resolution)
                obstacle_y = int(obstacle_y / map_resolution)

                obstacle_x_coords.append(obstacle_x)
                obstacle_y_coords.append(obstacle_y)
                obstacles.append((obstacle_x, obstacle_y))

                free_cells = self.raytrace([int((pose.pose.position.x - map_origin.position.x) / map_resolution), int((pose.pose.position.y - map_origin.position.y) / map_resolution)], [obstacle_x, obstacle_y])
                for free_cell in free_cells:
                    (free_x, free_y) = free_cell
                    self.add_to_map(grid_map, free_x, free_y, self.free_space)

        for obstacle in obstacles:
            (obs_x, obs_y) = obstacle
            self.add_to_map(grid_map, obs_x, obs_y, self.occupied_space)

        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min(obstacle_x_coords)
        # The minimum y index in 'grid_map' that has been updated
        update.y = min(obstacle_y_coords)
        # Maximum x index - minimum x index + 1
        update.width = max(obstacle_x_coords) - min(obstacle_x_coords) + 1
        # Maximum y index - minimum y index + 1
        update.height = max(obstacle_y_coords) - min(obstacle_y_coords) + 1
        # The map data inside the rectangle, in h-major order.
        
        update.data = []
        for h in range(update.height):
            for w in range(update.width):
                update.data.append(grid_map.__getitem__([h, w]))

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
        """

        map_width = grid_map.get_width()
        map_height = grid_map.get_height()
        map_origin = grid_map.get_origin()

        for w in range(map_width):
            for h in range(map_height):
                cell_x = int(map_origin.position.x + w)
                cell_y = int(map_origin.position.y + h)

                if self.is_in_bounds(grid_map, cell_x, cell_y):
                    if grid_map[cell_x, cell_y] == self.occupied_space:
                        expanded_cells = self.expansion([cell_x, cell_y])
                        for expanded_cell in expanded_cells:
                            if grid_map[expanded_cell[0], expanded_cell[1]] != self.occupied_space:
                                self.add_to_map(grid_map, expanded_cell[0], expanded_cell[1], self.c_space)

        # Return the inflated map
        return grid_map