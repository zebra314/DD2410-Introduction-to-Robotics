#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Ying Pei Lin
# {student id}
# yplin@kth.se

from dubins import *
import numpy as np
import heapq

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
    x, y = car.x0, car.y0
    xl, yl, thetal, phil, tl = [x], [y], [0.0], [], [0.0]
    phi = 0.1
    for _ in range(2000):
      xn, yn, thetan = step(car, xl[-1], yl[-1], thetal[-1], phi, dt=0.1)
      xl.append(xn)
      yl.append(yn)
      thetal.append(thetan)
      phil.append(phi)
      tl.append(tl[-1] + 0.1)
    return phil, tl

  # Drive the car along the path
  theta = 0.0
  phi_lst = []
  for i in range(len(path) - 1):
    x_curr, y_curr = path[i].x, path[i].y
    x_nxt, y_nxt = path[i + 1].x, path[i + 1].y
    phi = np.arctan2(y_nxt - y_curr, x_nxt - x_curr) - theta
    phi = np.clip(phi, -np.pi/4, np.pi/4)
    _, _, theta = step(car, x_curr, y_curr, theta, phi, dt=0.1)
    phi_lst.append(phi)

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
    self.open_set = self.PriorityQueue()

    # The set of nodes already evaluated
    self.closed_set = set()

    # Init nodes
    self.target_nd = self.Node(car.xt, car.yt, None, 0, 0)
    self.current_nd = self.Node(car.x0, car.y0, None, 0, self.get_dist(car.x0, car.y0, car.xt, car.yt))
    self.open_set.push(self.current_nd)
  
  def evaluate_node(self):
    while self.open_set.empty() == False:
      self.current_nd = self.open_set.pop()
      self.closed_set.add(self.current_nd)

      # print("Current: {:.2f}, {:.2f}".format(self.current_nd.x, self.current_nd.y))
      
      # Reached the target
      if self.current_nd == self.target_nd:
        print("Reached the target")
        # print("Target: {:.2f}, {:.2f}".format(self.target_nd.x, self.target_nd.y))
        # print("Current: {:.2f}, {:.2f}".format(self.current_nd.x, self.current_nd.y))
        path = self.get_track_path()
        return path

      neighbors = self.get_neighbors(self.current_nd)
      for neighbor in neighbors:
        if neighbor in self.closed_set:
          continue
        if neighbor not in self.open_set or neighbor.g < self.current_nd.g:
          neighbor.parent = self.current_nd
          self.open_set.push(neighbor)

  def get_track_path(self):
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
    
    def __lt__(self, other):
      return self.f < other.f

  class PriorityQueue:
    def __init__(self):
      self.queue = []
      self.set = set()

    def push(self, node):
      heapq.heappush(self.queue, node)
      self.set.add(node)

    def pop(self):
      node = heapq.heappop(self.queue)
      self.set.remove(node)
      return node

    def __contains__(self, node):
      return node in self.set

    def empty(self):
      return len(self.queue) == 0