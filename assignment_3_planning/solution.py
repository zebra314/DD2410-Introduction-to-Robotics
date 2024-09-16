#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Ying Pei Lin
# {student id}
# yplin@kth.se

from dubins import *
import numpy as np
from time import sleep

# Constants
car_resolution = 0.09 # rad, about 5 degrees
dt = 0.05

def solution(car):
  theta_lst = [0.0]  # Heading angle
  phi_lst = []       # Steering angle, -pi/4 <= phi <= pi/4
  time_lst = [0.0]

  astar = AStar(car, car_resolution)

  path = astar.evaluate_node()

  if path is None:
    print("No path found")
    return

  # Path to phi
  for i in range(len(path) - 1):
    node1 = path[i]
    node2 = path[i + 1]
    phi = 0
    phi_lst.append(phi)
    theta_lst.append(phi)

  for i in range(len(phi_lst)):
    time_lst.append(time_lst[-1] + dt)

  return phi_lst, time_lst

class AStar:
  def __init__(self, car, car_resolution):
    # Init map
    self.car_resolution = car_resolution
    self.x_min = car.xlb
    self.x_max = car.xub
    self.y_min = car.ylb
    self.y_max = car.yub

    # List of obstacles
    # [[x1, y1, r1], [x2, y2, r2], ...]
    self.obs = car.obs

    # The set of nodes to be evaluated
    self.open_set = self.NumpyPriorityQueue()

    # The set of nodes already evaluated
    self.closed_set = set()

    # Init nodes
    self.target_nd = self.Node(car.xt, car.yt, None, 0, 0)
    init_nd = self.Node(car.x0, car.y0, None, 0, self.get_dist(car.x0, car.y0, car.xt, car.yt))
    self.open_set.push(init_nd)
  
  def evaluate_node(self):
    while self.open_set.empty() == False:

      self.current_nd = self.open_set.pop()
      # print(f"Current node: {self.current_nd.x}, {self.current_nd.y}")
      self.closed_set.add(self.current_nd)

      # Reached the target
      if self.current_nd == self.target_nd:
        print("Reached the target ({}, {})".format(self.target_nd.x, self.target_nd.y))
        print("Current node: ({}, {})".format(self.current_nd.x, self.current_nd.y))
        path = self.get_path()
        return path
      
      neighbors = self.get_neighbors(self.current_nd)
      for neighbor in neighbors:
        if neighbor in self.closed_set:
          continue
        if neighbor not in self.closed_set or neighbor.g < node.g:
          neighbor.parent = self.current_nd
          self.closed_set.add(neighbor)
          self.open_set.push(neighbor)

  def get_path(self):
    path = []
    node = self.current_nd
    while node.parent != None:
      path.append(node)
      node = node.parent
    path.append(node)
    path.reverse()
    return path


  def get_dist(self, *args):
    if len(args) == 2 and all(isinstance(arg, Node) for arg in args):
      node1, node2 = args
      x1, y1 = node1.x, node1.y
      x2, y2 = node2.x, node2.y
    elif len(args) == 4 and all(isinstance(arg, (int, float)) for arg in args):
      x1, y1, x2, y2 = args
    else:
      raise ValueError("Invalid arguments. Pass either two Node objects or four numbers.")

    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

  def get_neighbors(self, node):
    neighbors = []
    for rad in np.linspace(-np.pi/4, np.pi/4, num=int((np.pi/2)/self.car_resolution) + 1):
      rad = round(rad, 3)
      new_x = node.x + np.cos(rad) * self.car_resolution
      new_y = node.y + np.sin(rad) * self.car_resolution

      if self.check_collision(new_x, new_y) == False:
        new_node = self.Node(new_x, new_y, node, 
                        node.g + self.get_dist(node.x, node.y, new_x, new_y), 
                        self.get_dist(new_x, new_y, self.target_nd.x, self.target_nd.y))
        neighbors.append(new_node)
    return neighbors

  def check_collision(self, x, y):
    if x < self.x_min or x > self.x_max or \
       y < self.y_min or y > self.y_max:
      return True
    for obs in self.obs:
      if self.get_dist(x, y, obs[0], obs[1]) < obs[2]:
        return True
    return False

  class Node:
    def __init__(self, x, y, parent, g, h):
      # Pos
      self.x = round(x, 1)
      self.y = round(y, 1)

      # Parent node
      self.parent = parent

      # Cost
      self.g = round(g, 1)
      self.h = round(h, 1)
      self.f = round((g + h), 1)
    
    def __eq__(self, other):
      if other is None or not isinstance(other, AStar.Node):
        return False
      return (self.x, self.y) == (other.x, other.y)

    def __hash__(self):
      return hash((self.x, self.y))
    
  class NumpyPriorityQueue:
    def __init__(self):
      self.queue = np.array([], dtype=float).reshape(0, 2)

    def push(self, node):
      # Add a new element and sort the array based on priority
      new_element = np.array([[node.f, node]])
      self.queue = np.vstack((self.queue, new_element))
      self.queue = self.queue[self.queue[:, 0].argsort()]  # Sort by the first column (priority)

    def pop(self):
      # Remove and return the element with the highest priority (lowest value)
      if self.queue.size == 0:
        return None
      element = self.queue[0, :]
      self.queue = np.delete(self.queue, 0, axis=0)
      return element[1]
    
    def empty(self):
      return self.queue.size == 0