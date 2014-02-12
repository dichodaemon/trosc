#!/usr/bin/env python

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt

from car_navigation_msgs.msg import Obstacle, Obstacles
from kalman_prediction_msg.msg import ObstaclePrediction, ObstaclesPrediction



observation_x = 0.0
observation_y = 0.0
speed = 0.0
x_pre = np.matrix('0. 0. 0. 0.').T 
P = np.matrix(np.eye(4))



pred_pub = rospy.Publisher( "prediction", ObstaclesPrediction)


def kalman(x, P, measurement, R, motion, Q, F, H):

    # UPDATE x, P based on measurement m    
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R  # residual convariance
    K = P * H.T * S.I    # Kalman gain
    x = x + K*y
    I = np.matrix(np.eye(4.0)) # identity matrix
    P = (I - K*H)*P

    # x, P based on motion
    x = F*x + motion
    P = F*P*F.T + Q

    return x, P


def kalman_2d(obs_x, obs_y, x, P):
  motion = np.matrix('0. 0. 0. 0.').T
  Q = np.matrix(np.eye(4)) # its r in dizan code
  
  F = np.matrix('''1. 0. 1. 0.;
  0. 1. 0. 1.;
  0. 0. 1. 0.;
  0. 0. 0. 1.
  ''')
  
  H = np.matrix('''1. 0. 0. 0.;
  0. 1. 0. 0.''')
  
  #x = np.matrix('0. 0. 0. 0.').T 
  #P = np.matrix(np.eye(4))
  R = np.eye( 2 ) * 0.05
  
  
  
  plt.ion()
  plt.show()
  plt.xlim((-2000, 2000))
  plt.ylim([-550, 550])
  
  for n in range(len(obs_x)):
    observation = (obs_x[n], obs_y[n])
    x, P = kalman(x, P, observation, R, motion, Q, F, H)
    plt.plot(x[0], x[1], 'ro')
    plt.plot(obs_x[n], obs_y[n], 'go')
    plt.draw()
    time.sleep(0.05)
  return x, P
  



def obstacledata( o ):
  global observation_x
  global observation_y
  global speed 
  for oi in o.obstacles:
    observation_x = oi.pose.x
    observation_y = oi.pose.y
    speed = oi.speed
  


def kalman_prediction():
  global x_pre
  global P
  
  obs_x = [observation_x]
  obs_y = [observation_y]
  #print "data ", x_pre, P
 
  x_pre, P = kalman_2d(obs_x, obs_y, x_pre, P)
  print obs_x, obs_y, speed, x_pre[2], x_pre[3]
  
  #ROS PUBLISHING DRAMA
  obs = ObstaclesPrediction()
  ob = ObstaclePrediction()
  ob.pose.x = x_pre[0]
  ob.pose.y = x_pre[1]
  ob.pose.y = np.arctan2(x_pre[3], x_pre[2])
  ob.xdot = x_pre[2]
  ob.ydot = x_pre[3]
  obs.obstaclesprediction.append( ob )
  pred_pub.publish( obs )
  
  
  
  
  
  

def measurment_data(event):
  
  #print 'For Storing the data'
  rospy.Subscriber("obstacles", Obstacles, obstacledata)
  kalman_prediction()

  

def listener():
  rospy.init_node('prediction', anonymous = True)
  rospy.Timer(rospy.Duration(.1), measurment_data)
  rospy.spin()

if __name__ == '__main__':
  listener()
