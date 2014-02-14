#!/usr/bin/env python

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from kalman import Kalman
from car_navigation_msgs.msg import Obstacle, Obstacles
from kalman_prediction_msg.msg import Prediction, Predictions, PredictionOneStep
import sys

pred_pub = rospy.Publisher( "prediction_new", Predictions)

count = 0
last_time = 0.0
timeHorizon=20

obstacle_dict = {}

#Kalman Variable Initialization
factor = 0.0002
p = np.eye( 4 ) * 1.
q = np.eye( 2 ) * 0.05


#Control error 
u = np.array([0.0, 0.0, 0.001, 0.0])

a = np.array( [
    [ 1., 0., 1., 0. ],
    [ 0., 1., 0., 1. ],
    [ 0., 0., 1., 0. ],
    [ 0., 0., 0., 1. ]
  ] )
  
c = np.array( [
    [ 1., 0., 0., 0. ],
    [ 0., 1., 0., 0. ]
  ] )




class default_noise( object ):
  def __init__( self, factor ):
    self.factor = factor
  def __call__( self, delta ):
    d3 = delta**3 * self.factor / 3.
    d2 = delta**2 * self.factor / 2.
    d  = delta * self.factor
    return np.array( [
      [d3, 0., d2, 0.],
      [0., d3, 0., d2],
      [d2, 0.,  d,  0],
      [0,  d2,  0,  d]
    ] )
    


def kalman_update(observation_x, observation_y, id_number, delta_t):
  
  observation = np.array( [observation_x, observation_y] )
  #Predict the mean and Sigma
  if delta_t != 0.0:
    obstacle_dict[id_number].predict(delta_t)
    
  #update the mean and sigma
  obstacle_dict[id_number].update(observation)



#Write ROS Module
def listener( o ):
  global timeHorizon
  global last_time
  global count 
  
  if count % 10 != 0:
    count += 1
    return  
  count += 1
  if last_time == 0.0:
    delta_t = 0.0
  else:
    delta_t =  rospy.get_time() - last_time
    
  sys.stderr.write( "delta: %f\n"  % ( delta_t ) )
  last_time =  rospy.get_time()
  
  r = default_noise(factor)

  for oi in o.obstacles:
    observation_x = oi.pose.x
    observation_y = oi.pose.y
    speed = oi.speed
    id_number = oi.id

    if not id_number in obstacle_dict:
      mean = np.array([observation_x, observation_y, 0.0, 0.0])
      obj = Kalman(mean, a, c, r, q, p)
      obstacle_dict[id_number] = obj
    else:
      kalman_update(observation_x, observation_y, id_number, delta_t )
  
  #Now for each obstacle at every t, predict the future state,(say for 20 timeStep) 
  preds = Predictions()
  
  for i in range(0, len(obstacle_dict)):
    pred = Prediction()
    pred.id = i
    pred.timeHorizon = timeHorizon
    
    mu = obstacle_dict[i].get_mean()
    sigma = obstacle_dict[i].get_p()
    
    for j in range(0, timeHorizon):
      predonestep = PredictionOneStep()
      mu = np.dot( a, mu )
      sigma = np.dot( np.dot( a, sigma ), np.transpose( a ) ) + r(delta_t)
      cov = sigma.reshape( 16 )
      
      predonestep.mean = mu
      predonestep.cov = cov
      pred.predictiononestep.append( predonestep )
    
    preds.prediction.append( pred )
  pred_pub.publish( preds )
      

if __name__ == '__main__':
  rospy.init_node('prediction', anonymous = True)
  rospy.Subscriber("obstacles", Obstacles, listener)
  rospy.spin()
  
