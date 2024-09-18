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
  
    self.open_set = dict() # nodes to be evaluated
    self.closed_set = dict() # nodes already evaluated

  def evaluate_node(self):
    self.plot_init()

    current_nd = self.Node(self.car.x0, self.car.y0, 0)
    current_nd.g = 0
    current_nd.h = self.get_dist(self.car.x0, self.car.y0, self.car.xt, self.car.yt)
    current_nd.f = current_nd.g + 2 * current_nd.h
    self.open_set[current_nd.idx] = current_nd

    while not len(self.open_set) == 0:
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
        if neighbor.idx in self.closed_set:
          continue
        elif neighbor.idx in self.open_set and neighbor.f < self.open_set[neighbor.idx].f:
          self.open_set[neighbor.idx] = neighbor
        elif neighbor.idx not in self.open_set or neighbor.g < current_nd.g:
          self.open_set[neighbor.idx] = neighbor

  def get_track_path(self, node):
    control_lst, time_lst = [], []
    while node.parent != None:
      time_lst.insert(0, node.time * self.time_unit)
      control_lst.insert(0, node.phi)

      if PLOT_ENABLED == True:
        self.ax.plot(node.x, node.y, 'go')

      node = node.parent
    time_lst.insert(0, 0)
    return control_lst, time_lst
  
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

    # phi_lst = [-0.78, -0.39, 0, 0.39, 0.78]
    phi_lst = [-0.78, 0, 0.78]

    for phi in phi_lst:
      nxt_node = self.get_nxt_node(node, phi) 
      
      if nxt_node is not None:
        nxt_node.phi = phi
        nxt_node.time = node.time + self.sim_times
        nxt_node.parent = node
        nxt_node.g = node.g + self.time_unit * self.sim_times
        nxt_node.h = self.get_dist(nxt_node.x, nxt_node.y, self.car.xt, self.car.yt)
        nxt_node.f = nxt_node.g + 2 * nxt_node.h
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
  
      if not self.check_reachable(x, y):
        node = self.Node(x, y, theta)
        self.closed_set[node.idx] = node
        return None

      while theta > np.pi:
        theta -= 2 * np.pi
      while theta < -np.pi:
        theta += 2 * np.pi

    nxt_node = self.Node(x, y, theta)
    return nxt_node

  def check_reachable(self, x, y):
    # Check collision
    for obs in self.car.obs:
      if self.get_dist(x, y, obs[0], obs[1]) <= obs[2] + 0.15:
        return False

    # Check boundary
    if self.car.xlb < x < self.car.xub and self.car.ylb < y < self.car.yub:
      return True
    return False

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