#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Ying Pei Lin
# {student id}
# yplin@kth.se

from dubins import *
import numpy as np

# Constants
car_resolution = 0.01 # rad
dt = 0.05

def solution(car):
  x_lst = [car.x0]
  y_lst = [car.y0]
  theta_lst = [0.0]  # Heading angle
  phi_lst = []       # Steering angle, -pi/4 <= phi <= pi/4
  time_lst = [0.0]

  nodes = Nodes(car, car_resolution)

  node = Node(car.x0, car.y0, 0, None, 0, get_dist(car.x0, car.y0, car.xt, car.yt))
  nodes.open_set.push(node)

  while nodes.open_set.empty() == False:
    node = nodes.open_set.pop()
    nodes.closed_set.add(node)

    # print("Current distance:", node.f)

    # Reached the target
    if [node.x, node.y] == [car.xt, car.yt]:
      break

    neighbors = nodes.get_neighbors(node)
    for neighbor in neighbors:
      if neighbor in nodes.closed_set:
        continue
      if neighbor not in nodes.closed_set or neighbor.f < node.f:
        neighbor.parent = node
        nodes.closed_set.add(neighbor)
        nodes.open_set.push(neighbor)
        
  print("Path found!")

  phi = 0.1
  for _ in range(4000):
    x_curr, y_curr, theta_curr = step(car, x_lst[-1], y_lst[-1], theta_lst[-1], phi, dt=dt)
    x_lst.append(x_curr)
    y_lst.append(y_curr)
    theta_lst.append(theta_curr)
    phi_lst.append(phi)
    time_lst.append(time_lst[-1] + dt)  

  return phi_lst, time_lst

def get_dist(x1, y1, x2, y2): 
  x_delta = x2 - x1
  y_delta = y2 - y1
  return (x_delta ** 2 + y_delta ** 2) ** 0.5

class Nodes:
  def __init__(self, car, car_resolution):
    self.car_resolution = car_resolution
    self.xt = car.xt
    self.yt = car.yt
    self.x_min = car.xlb
    self.x_max = car.xub
    self.y_min = car.ylb
    self.y_max = car.yub
    self.obs = car.obs # List of obstacles, obs = [[x1, y1, r1], [x2, y2, r2], ...]

    self.open_set = NumpyPriorityQueue()
    self.closed_set = set()

  def get_neighbors(self, node):
    neighbors = []
    for rad in np.arange(-np.pi/4, np.pi/4, self.car_resolution):
        new_x = node.x + np.cos(node.theta + rad) * self.car_resolution
        new_y = node.y + np.sin(node.theta + rad) * self.car_resolution
        if self.check_collision(new_x, new_y) == False:
          new_node = Node(new_x, new_y, rad, node, 
                      node.g + get_dist(node.x, node.y, new_x, new_y), 
                      get_dist(new_x, new_y, self.xt, self.yt))
          neighbors.append(new_node)
    return neighbors

  def check_collision(self, x, y):
    if x < self.x_min or x > self.x_max or \
       y < self.y_min or y > self.y_max:
      return True
    for obs in self.obs:
      if get_dist(x, y, obs[0], obs[1]) < obs[2]:
        return True
    return False

class Node:
  def __init__(self, x, y, theta, parent, g, h):
    self.x = x
    self.y = y
    self.theta = theta
    self.parent = parent
    self.g = g
    self.h = h
    self.f = g + h
    
class NumpyPriorityQueue:
  def __init__(self):
    self.queue = np.array([], dtype=float).reshape(0, 2)

  def push(self, node):
    # Add a new element and sort the array based on priority
    new_element = np.array([[node.f, node]])
    self.queue = np.vstack((self.queue, new_element))
    self.queue = self.queue[self.queue[:, 0].argsort()]  # Sort by the first column (priority)

    print(" ------------------------------------")
    print("size:", self.queue.size)
    print(" ------------------------------------")

  def pop(self):
    # Remove and return the element with the highest priority (lowest value)
    if self.queue.size == 0:
      return None
    element = self.queue[0, :]
    self.queue = np.delete(self.queue, 0, axis=0)
    return element[1]
  
  def empty(self):
    return self.queue.size == 0