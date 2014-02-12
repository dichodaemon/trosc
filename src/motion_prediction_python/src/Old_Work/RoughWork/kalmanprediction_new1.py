#!/usr/bin/env python

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from kalman import Kalman
from car_navigation_msgs.msg import Obstacle, Obstacles
from kalman_prediction_msg.msg import ObstaclePrediction, ObstaclesPrediction

pred_pub = rospy.Publisher( "prediction_new", ObstaclesPrediction)

observation_x = 0.0
observation_y = 0.0
speed = 0.0

#Kalman Variable Initialization
factor = 0.0002
delta = 0.1
mean = np.array([0.0, 0.0, 0.0, 0.0])
p = np.eye( 4 ) * 1.
q = np.eye( 2 ) * 0.25

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
    



#Write ROS Module
def obstacledata( o ):
  global observation_x
  global observation_y
  global speed 

  for oi in o.obstacles:
    observation_x = oi.pose.x
    observation_y = oi.pose.y
    speed = oi.speed
    print oi.pose.x
    print oi.pose.y

def kalman_prediction():
  
  global mean
  global p

  r = default_noise(factor)
  observation = np.array( [observation_x, observation_y] )
  
  
   #You need to publish the mean and covariance 
  cov_xy = p[0:2, 0:2]
  (eigVal, eigVec) = np.linalg.eig(cov_xy)
  obs = ObstaclesPrediction()
  ob = ObstaclePrediction()
  ob.id = 0
  ob.pose.x = mean[0]
  ob.pose.y = mean[1]
  ob.pose.theta = np.arctan2(mean[3], mean[2])
  ob.xdot = mean[2]
  ob.ydot = mean[3]
  ob.eigx = eigVal[0]
  ob.eigy = eigVal[1]
  ob.tmajor = np.arctan2(eigVec[0][1],eigVec[0][0])
  ob.tminor = np.arctan2(eigVec[1][1],eigVec[1][0])
  obs.obstaclesprediction.append( ob )
  pred_pub.publish( obs )
  
  
  
  #Initialize the kalman variables 
  obj = Kalman(mean, a, c, r, q, p)
  
  #Predict the mean and Sigma
  obj.predict(delta)
  mean = obj.get_mean()
  p = obj.get_p()
  
  
  #update the mean and sigma
  obj.update(observation)
  mean = obj.get_mean()
  p = obj.get_p()

  

  
  

def measurment_data(event):
  rospy.Subscriber("obstacles", Obstacles, obstacledata)
  kalman_prediction()
  #Now for a time horizon you have to predict the mean and sigma and also plot the covariance ellipse for that.
  
  
def H_prediction(event):
  
  global delta
  global mean
  global p
  r = default_noise(factor)
  delta_t = 0.1
  
  ts = rospy.Time.from_sec(time.time())
  nanoseconds_old = ts.to_nsec()
  nanoseconds_old = nanoseconds_old + 2000000 #Adding 2x10^8 ns for 4Hz  
  nanoseconds_new = 0

  
  # here we can change delta value w.r.t time
  
  while(nanoseconds_new < nanoseconds_old):
    tsn = rospy.Time.from_sec(time.time())
    nanoseconds_new = tsn.to_nsec()
    mean = np.dot( a, mean )
    p = np.dot( np.dot( a, p ), np.transpose( a ) ) + r(delta_t)
    delta_t = delta_t + 0.0001
    
    #You need to publish the mean and covariance 
    '''cov_xy = p[0:2, 0:2]
    (eigVal, eigVec) = np.linalg.eig(cov_xy)
    obs = ObstaclesPrediction()
    ob = ObstaclePrediction()
    ob.id = 0
    ob.pose.x = mean[0]
    ob.pose.y = mean[1]
    ob.pose.theta = np.arctan2(mean[3], mean[2])
    ob.xdot = mean[2]
    ob.ydot = mean[3]
    ob.eigx = eigVal[0]
    ob.eigy = eigVal[1]
    ob.tmajor = np.arctan2(eigVec[0][1],eigVec[0][0])
    ob.tminor = np.arctan2(eigVec[1][1],eigVec[1][0])
    obs.obstaclesprediction.append( ob )
    pred_pub.publish( obs )'''
  

  

def listener():
  rospy.init_node('prediction_new', anonymous = True)
  rospy.Timer(rospy.Duration(.1), measurment_data)
  rospy.Timer(rospy.Duration(1), H_prediction)
  rospy.spin()

if __name__ == '__main__':
  listener()
  
  
  
  
  
  
  '''plt.ion()
  plt.show()
  plt.xlim((-2000, 2000))
  plt.ylim([-550, 550])
   plt.plot(mean[0], mean[1], 'ro')
  plt.plot(observation[0], observation[1], 'go')
  plt.draw()'''
  
  
    '''obs = ObstaclesPrediction()

  cov_xy = covarianceList[id_number].covariance_matrix[0:2, 0:2]
  (eigVal, eigVec) = np.linalg.eig(cov_xy)
  ob = ObstaclePrediction()
  ob.id = id_number
  ob.pose.x = meanList[id_number].mean_array[0]
  ob.pose.y = meanList[id_number].mean_array[1]
  ob.pose.theta = np.arctan2(meanList[id_number].mean_array[3], meanList[id_number].mean_array[2])
  ob.xdot = meanList[id_number].mean_array[2]
  ob.ydot = meanList[id_number].mean_array[3]
  ob.eigx = eigVal[0]
  ob.eigy = eigVal[1]
  ob.tmajor = np.arctan2(eigVec[0][1],eigVec[0][0])
  ob.tminor = np.arctan2(eigVec[1][1],eigVec[1][0])
  obs.obstaclesprediction.append( ob )
  pred_pub.publish( obs )'''
