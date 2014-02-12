#!/usr/bin/env python



import rospy
import numpy as np
from kalman import Kalman


from car_navigation_msgs.msg import Obstacle, Obstacles
from kalman_prediction_msg.msg import ObstaclePrediction, ObstaclesPrediction

factor = 0.0002
delta = 0.1
callcount = 0
pred_pub = rospy.Publisher( "prediction", ObstaclesPrediction)
pred_speed = 0.0
pred_mean = [0.0, 0.0, 0.0, 0.0]


xpre = np.zeros((1,11))
ypre = np.zeros((1,11))
xdot = np.zeros((1,11))
ydot = np.zeros((1,11))

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
q = np.eye( 2 ) * 0.25
p = np.eye( 4 ) * 1.



updateList = []

class update_covariance(object):
  def __init__(self, covariance_matrix):
    self.covariance_matrix = covariance_matrix
  
    



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
    


def init_covariance():
  for x in range(0, 11):
    updateList.append(update_covariance(np.eye(4)))




def obstacledata( o ):
  global callcount
  global pred_pub
  global pred_mean
  global delta
  global pred_speed
   
  callcount = callcount + 1
  
  if callcount == 1:
    init_covariance()
  else:
    obs = ObstaclesPrediction()
    r = default_noise(factor)
    
    for oi in o.obstacles:
      
      mean = np.array([oi.pose.x, oi.pose.y, xdot[0][oi.id], ydot[0][oi.id]])
      observation = np.array( [1.0, 1.0] ) 
      print "for mean ", xdot[0][oi.id]
      

        
      obj = Kalman(mean, a, c, r, q, updateList[oi.id].covariance_matrix)
     
      #prediction step
      obj.predict(delta)
      pred_mean = obj.get_mean()
      pred_sigm = obj.get_p()
      pred_speed = np.sqrt(pred_mean[2]**2 + pred_mean[3]**2)
       
     
      #update step
      obj.update(observation)
      updated_mean = obj.get_mean()
      updated_sigm = obj.get_p()
      #print "mean ", updated_mean
     
     
      #changing the noise covariance matrix
      delta = delta + 0.1
     
      #changing mean and sigma to updated mean and sigma
      mean = obj.get_mean()
      updateList[oi.id].covariance_matrix = obj.get_p()
      obj.set_mean(mean)
      obj.set_p(updateList[oi.id].covariance_matrix )
      
    print "Original Speed", oi.speed
    print "Predicted Speed ", pred_speed
    xpre[0][oi.id] = pred_mean[0] #x updated
    ypre[0][oi.id] = pred_mean[1] #y updated
    xdot[0][oi.id] = pred_mean[2] #xdot updated
    ydot[0][oi.id] = pred_mean[3] #ydot updated ''' 
    print "velocity ", xdot[0][oi.id]
      
       
    
    
      
    
    
    
    #I need to publish xpred, ypred, theta_major, eigen_valuex and eigen_valuey
    '''covxy  = updateList[oi.id].covariance_matrix[0:2, 0:2]
     (eigVal, eigVec) = np.linalg.eig(covxy)
     
    
     
     ob = ObstaclePrediction()
     ob.id = oi.id
     ob.pose.x = xpre[0][oi.id]
     ob.pose.y = ypre[0][oi.id]
     ob.pose.theta = np.arctan2(ydot[0][oi.id], xdot[0][oi.id])
     ob.xdot = xdot[0][oi.id]
     ob.ydot = ydot[0][oi.id]
     ob.eigx = eigVal[0]
     ob.eigy = eigVal[1]
     ob.tmajor = np.arctan2(eigVec[0][1],eigVec[0][0])
     ob.tminor = np.arctan2(eigVec[1][1],eigVec[1][0])
     obs.obstaclesprediction.append( ob )
    pred_pub.publish( obs )'''
    #obs_x = [45.8475990295, 46.9746246338, 54.9658088684, 70.5696334839, 93.7653656006, 112.042449951, 132.032104492, 153.720230103, 157.497772217, 162.283874512 ]
  #obs_y = [-2.66391515732, -2.63587236404, -2.43866562843, -1.99639415741, -1.32803344727, -0.838838577271, -0.336437165737, 0.154014527798, 0.235280692577, 0.347968041897]
     
     
     
     

    
    
    
    


if __name__ == '__main__':
  rospy.init_node('kalmanfilter', anonymous=True)
  rospy.Subscriber("obstacles", Obstacles, obstacledata)
  rospy.spin()
