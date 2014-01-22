#!/usr/bin/env python
import rospy
import numpy as np

from car_navigation_msgs.msg import Control, Obstacles, Status
from nav_msgs.msg import OccupancyGrid

import sys
import os

BASE_DIR = os.path.abspath( os.path.join( os.path.dirname( __file__ ), ".." ) )
path     = os.path.abspath( os.path.join( BASE_DIR, "python" ) )
sys.path.append( path )

from math import *
import ast
import belly

cost_pub = rospy.Publisher('costmap', OccupancyGrid)

class Params( object ): pass

def param( name, default = None ):
  if default != None:
    return rospy.get_param( "/" + rospy.get_name() + "/"  + name, default )
  else:
    return rospy.get_param( "/" + rospy.get_name() + "/"  + name )

def get_params():
  result = Params()
  result.feature_type = param( "feature_type" )
  result.feature_params = ast.literal_eval( param( "feature_params" ) )
  result.weights = np.array( ast.literal_eval( param( "weights" ) ), dtype = np.float64 )
  result.speed = param( "speed" )
  result.cell_size = param( "cell_size" )
  result.x1   = param( "x1" )
  result.x2   = param( "x2" )
  result.y1   = param( "y1" )
  result.y2   = param( "y2" )
  result.max_msg_age = param( "max_msg_age" )
  return result

def publish_costmap( costs, cell_size, x1, y1 ):
  cc = costs[0] * 1.0
  cc *= 100.0 / np.max( cc )
  cc = cc.astype( np.int8 )
  w, h = cc.shape
  c = np.reshape( cc, w * h )

  ocg = OccupancyGrid()
  ocg.header.stamp = rospy.Time.now()
  ocg.header.frame_id = "world"
  ocg.data = c
  ocg.info.resolution = cell_size
  ocg.info.width = h
  ocg.info.height = w
  ocg.info.origin.position.x = x1
  ocg.info.origin.position.y = y1
  cost_pub.publish( ocg )


def compute_costmap( weights, feature_type, feature_params, x1, y1, x2, y2, cell_size, other, speed ):
  # Build planning objects
  convert = belly.convert( { "x1": x1, "y1": y1, "x2": x2, "y2": y2 }, cell_size )
  features = belly.features.__dict__[feature_type]
  compute_features = features.compute_features( convert, **feature_params )
  compute_costs = belly.features.compute_costs( convert )

  # Compute features and costs
  f = compute_features( speed, other )
  costs = compute_costs( f, weights )
  publish_costmap( costs, cell_size, x1, y1 )



def obstacle_callback( data ):
  parms = get_params()
  age = rospy.get_rostime().to_sec() - data.header.stamp.to_sec()
  if age >  parms.max_msg_age : 
    sys.stderr.write( "Time: %f; Header: %f; Age: %f\n" % ( rospy.get_rostime().to_sec(), data.header.stamp.to_sec(), age ) )
    return

  other = []
  for a in data.obstacles:
    v = np.array( [a.pose.x, a.pose.y, a.speed * cos( a.pose.theta ), a.speed * sin( a.pose.theta )], dtype = np.float32 )
    other.append( v )
  other = np.array( other )


  compute_costmap( 
    parms.weights, parms.feature_type, parms.feature_params, 
    parms.x1, parms.y1, parms.x2, parms.y2, parms.cell_size, 
    other, parms.speed 
  )

def listener():
  rospy.init_node( 'compute_features' )
  p = get_params()
  rospy.Subscriber( "trackcordinates", Obstacles, obstacle_callback, queue_size = 1 )
  rospy.spin()


if __name__ == '__main__':
  listener()
