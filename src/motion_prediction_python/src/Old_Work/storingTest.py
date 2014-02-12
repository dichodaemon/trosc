#!/usr/bin/env python



import numpy as np

updateList = []
no_of_obstacle = 10

class update_covariance(object):
  def __init__(self, covariance_matrix):
    self.covariance_matrix = covariance_matrix


def init_covariance():
  for x in range(0, 11):
    updateList.append(update_covariance(np.zeros(4)))
    
    

init_covariance()

for i in range(1,no_of_obstacle+1):
  print updateList[i].covariance_matrix[0]
  print updateList[i].covariance_matrix[1]
