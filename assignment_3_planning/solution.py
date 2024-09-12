#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {student full name}
# {student id}
# {student email}

from dubins import *

def solution(car):

  '''
  Your code below
  '''

  # initial state
  x, y = car.x0, car.y0
  theta = 0

  # arbitrary control
  phi = 0.2

  # compute next state after 0.01 seconds
  xn, yn, thetan = step(car, x, y, theta, phi)

  # assemble path
  controls, times = [phi], [0, 0.01]

  '''
  Your code above
  '''

  return controls, times