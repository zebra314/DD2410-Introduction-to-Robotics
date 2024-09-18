#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Ying Pei Lin
# {student id}
# yplin@kth.se

PLOT_ENABLED = True

from dubins import *
import numpy as np
from copy import copy

if PLOT_ENABLED == True:
  import matplotlib.pyplot as plt

def solution(car):
  astar = AStar(car)
  control_lst, time_lst = astar.evaluate_node()

  return control_lst, time_lst

class AStar:
  def __init__(self, car):
    self.car = car
    self.sim_times = 45
    self.time_unit = 0.01
    self.target_threshold = 1.0

    self.heuristic_weight = 1.3

    self.open_set = dict() # nodes to be evaluated
    self.closed_set = dict() # nodes already evaluated

  def evaluate_node(self):
    """
    Implement the A* algorithm to find the path from the start node to the target node

    @input:
      None, use the class variables set in the constructor

    @return:
      control_lst: list of steering angles
      time_lst: list of simulation time
    """
    self.plot_init()

    # Init start node
    current_nd = self.Node(self.car.x0, self.car.y0, 0)
    current_nd.g = 0
    current_nd.h = self.get_dist(self.car.x0, self.car.y0, self.car.xt, self.car.yt)
    current_nd.f = current_nd.g + 2 * current_nd.h
    self.open_set[current_nd.idx] = current_nd

    while not len(self.open_set) == 0:

      # Get the node with the lowest f value from the open set
      # and move it to the closed set
      current_idx = min(self.open_set, key=lambda x: self.open_set[x].f)
      current_nd = self.open_set[current_idx]
      del self.open_set[current_idx]
      self.closed_set[current_idx] = current_nd

      # Reached the target
      if self.get_dist(current_nd.x, current_nd.y, self.car.xt, self.car.yt) < self.target_threshold:
        control_lst, time_lst = self.get_track_path(current_nd)
        self.plot_close()
        return control_lst, time_lst

      self.plot_point(current_nd.x, current_nd.y)

      neighbors = self.get_neighbors(current_nd)
      if len(neighbors) == 0:
        continue
      
      for neighbor in neighbors:

        # Skip if the neighbor is already evaluated
        if neighbor.idx in self.closed_set:
          continue
      
        # Discover a new node
        elif neighbor.idx not in self.open_set:
          self.open_set[neighbor.idx] = neighbor  

        # If the neighbor is already in the open set and has a lower g value
        elif neighbor.idx in self.open_set and neighbor.g < self.open_set[neighbor.idx].g:
          self.open_set[neighbor.idx] = neighbor

  def get_track_path(self, node):
    """
    Trace back the path from the target node to the start node

    @input:
      node: current node

    @return:
      control_lst: list of steering angles
      time_lst: list of simulation time
    """
    control_lst, time_lst = [], []
    while node.parent != None:
      time_lst.insert(0, node.time * self.time_unit)
      control_lst.insert(0, node.phi)

      if PLOT_ENABLED == True:
        self.ax.plot(node.x, node.y, 'go')

      node = node.parent
    time_lst.insert(0, 0)
    return control_lst, time_lst

  def get_neighbors(self, node):
    """
    Get the neighbors of the current node

    @input:
      node: current node
    
    @return:
      neighbors: list of nodes that are reachable from the
                  current node with a steering angle of -pi/4, 0, pi/4
    """
    neighbors = []

    # phi_lst = [-0.78, -0.39, 0, 0.39, 0.78]
    phi_lst = [-0.78, 0, 0.78]

    for phi in phi_lst:
      nxt_node = self.get_nxt_node(node, phi) 
      
      # If the position is reachable, initialize the new node
      if nxt_node is not None:
        nxt_node.phi = phi
        nxt_node.time = node.time + self.sim_times
        nxt_node.parent = node
        nxt_node.g = node.g + self.time_unit * self.sim_times
        nxt_node.h = self.get_dist(nxt_node.x, nxt_node.y, self.car.xt, self.car.yt)
        nxt_node.f = nxt_node.g + self.heuristic_weight * nxt_node.h
        neighbors.append(nxt_node)
    return neighbors

  def get_nxt_node(self, node, phi):
    """
    @input:
      node: current node
    
    @return:
      x, y, theta: next position and heading angle
      control_lst: updated list of steering angles
      time_lst: updated list of simulation time
    """
    x, y, theta = node.x, node.y, node.theta

    for _ in range(self.sim_times):
      x, y, theta = step(self.car, x, y, theta, phi, dt=self.time_unit)
  
      # Check if the position is reachable
      # If not, add the node to the closed set and return None
      if not self.check_reachable(x, y):
        node = self.Node(x, y, theta)
        self.closed_set[node.idx] = node
        return None

      # Normalize theta
      while theta > np.pi:
        theta -= 2 * np.pi
      while theta < -np.pi:
        theta += 2 * np.pi

    nxt_node = self.Node(x, y, theta)
    return nxt_node

  def check_reachable(self, x, y):
    """
    Check if the input position is reachable

    @input:
      x, y: position

    @return:
      Boolean: True if the position is reachable, False otherwise
    """
    # Check collision
    for obs in self.car.obs:
      if self.get_dist(x, y, obs[0], obs[1]) <= obs[2] + 0.15:
        return False

    # Check boundary
    if self.car.xlb < x < self.car.xub and self.car.ylb < y < self.car.yub:
      return True
    return False
  
  def get_dist(self, x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
  
  def plot_init(self):
    if PLOT_ENABLED == True:
      plt.ion()
      self.fig, self.ax = plt.subplots()
      self.ax.set_xlim([self.car.xlb, self.car.xub])
      self.ax.set_ylim([self.car.ylb, self.car.yub])
      self.ax.plot(self.car.xt, self.car.yt, 'bo')
      for obs in self.car.obs:
          obstacle = plt.Circle((obs[0], obs[1]), obs[2], color='blue', label='Obstacle')
          plt.gca().add_artist(obstacle)
  
  def plot_close(self):
    if PLOT_ENABLED == True:
      plt.pause(1)
      plt.close('all')
  
  def plot_point(self, x, y):
    if PLOT_ENABLED == True:
      self.ax.plot(x, y, 'ro')
      plt.pause(0.001)

  class Node:
    def __init__(self, x, y, theta):
      # Resolutions
      self.linear_res = 0.3
      self.angular_res = 2.0 * np.pi / 6.0

      # Index
      x_idx = int(x / self.linear_res)
      y_idx = int(y / self.linear_res)
      theta_idx = int((theta % (2.0 * np.pi)) / self.angular_res)
      self.idx = (x_idx, y_idx, theta_idx)

      # Pos
      self.x = x
      self.y = y  
      self.theta = theta

      # Parent node
      self.parent = None

      # Cost
      self.g = 0
      self.h = 0
      self.f = 0

      self.phi = 0
      self.time = 0