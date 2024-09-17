#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Ying Pei Lin
# {student id}
# yplin@kth.se

from dubins import *
import numpy as np
import heapq
from copy import copy

def solution(car):
  theta_lst = [0.0]  # Heading angle
  phi_lst = []       # Steering angle, -pi/4 <= phi <= pi/4
  time_lst = [0.0]

  astar = AStar(car)
  path = astar.evaluate_node()

  if path is None:
    # print("No path found")
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
  def __init__(self, car):
    # Init navigation settings
    self.car = car
    self.time_resolution = 0.01
    self.target_threshold = 0.01
  
    # The set of nodes to be evaluated
    self.open_set = self.PriorityQueue()

    # The set of nodes already evaluated
    self.closed_set = set()

    # Init nodes
    self.current_nd = self.Node(car.x0, car.y0, 0, None, 0, self.get_dist(car.x0, car.y0, car.xt, car.yt), [], [0.0])
    self.open_set.push(self.current_nd)
  
  def evaluate_node(self):
    while self.open_set.empty() == False:
      self.current_nd = self.open_set.pop()
      self.closed_set.add(self.current_nd)
      
      # Reached the target
      if self.get_dist(self.current_nd.x, self.current_nd.y, self.car.xt, self.car.yt) < self.target_threshold:
        print("Reached the target")
        path = self.get_track_path()
        return path

      neighbors = self.get_neighbors(self.current_nd)

      if len(neighbors) == 0:
        continue

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

  def get_dist(self, x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

  def get_neighbors(self, node):
    """
    @input:
      node: current node
    
    @return:
      neighbors: list of nodes that are reachable from the
                  current node with a steering angle of -pi/4, 0, pi/4
    """
    neighbors = []
    for phi in [-0.78, -0.68, -0.59, -0.49, -0.39, -0.29, -0.20, -0.10, 0, 0.10, 0.20, 0.29, 0.39, 0.49, 0.59, 0.68, 0.78]:
    # for phi in [-0.78, -0.59, -0.39, -0.20, 0, 0.20, 0.39, 0.59, 0.78]:
    # for phi in [-0.78, -0.50, -0.20, 0, 0.20, 0.50, 0.78]:
    # for phi in [-0.78, -0.39, 0, 0.39, 0.78]:
      x_nxt, y_nxt, theta_nxt, control_lst, time_lst = self.get_nxt_state(
        node.x, node.y, node.theta, phi, copy(node.control_lst), copy(node.time_lst)
      )
      if x_nxt is not None:
        new_node_g = node.g + self.get_dist(node.x, node.y, x_nxt, y_nxt)
        new_node_h = self.get_dist(x_nxt, y_nxt, self.car.xt, self.car.yt)
        new_node = self.Node(
          x_nxt,
          y_nxt, 
          theta_nxt,
          node,
          new_node_g,
          new_node_h,
          control_lst,
          time_lst
        )        
        neighbors.append(new_node)
    return neighbors

  def get_nxt_state(self, x, y, theta, phi, control_lst, time_lst):
    """
    @input:
      x, y, theta: current position and heading angle
      phi: steering angle
      control_lst: list of steering angles
      time_lst: list of simulation time
    
    @return:
      x, y, theta: next position and heading angle
      control_lst: updated list of steering angles
      time_lst: updated list of simulation time
    """

    # If the car is turning, increase the simulation time
    for sim_time in range(4 if phi != 0 else 2):
      x, y, theta = step(self.car, x, y, theta, phi)
      control_lst.append(phi)
      time_lst.append(time_lst[-1] + self.time_resolution)

      while theta > np.pi:
        theta -= 2 * np.pi
      while theta < -np.pi:
        theta += 2 * np.pi

    if not self.check_reachable(x, y):
      return None, None, None, None, None
    return x, y, theta, control_lst, time_lst

  def check_reachable(self, x, y):
    # Check collision
    for obs in self.car.obs:
      if self.get_dist(x, y, obs[0], obs[1]) <= obs[2]:
        return False

    # Check boundary
    if self.car.xlb < x < self.car.xub and self.car.ylb < y < self.car.yub:
      return True
    else:
      return False

  class Node:
    def __init__(self, x, y, theta, parent, g, h, control_lst, time_lst):
      # Pos
      # self.x = round(x * 10, 2) / 10
      # self.y = round(y * 10, 2) / 10
      self.x = round(x , 3)
      self.y = round(y , 3)
      self.theta = round(theta, 3)

      # Parent node
      self.parent = parent

      # Cost
      self.g = g
      self.h = h
      self.f = (g + h)

      self.control_lst = control_lst
      self.time_lst = time_lst
    
    def __eq__(self, other):
      if other is None or not isinstance(other, AStar.Node):
        return False
      return (self.x, self.y, self.theta) == (other.x, other.y, other.theta)

    def __hash__(self):
      return hash((self.x, self.y, self.theta))
    
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