import numpy as np
from math import *
import sys

def steering( 
  track_yaw, track_curvature, track_distance, track_width 
):
  yaw_error = track_yaw 
  #if next_curvature > 0.0:
    #track_distance -= track_width / 2 - 2.0
  #elif next_curvature < 0.0:
    #track_distance += track_width / 2 - 2.0
  #lane_error = atan2( track_distance, next_distance )
  #if track_curvature > 0.0:
    #track_distance -= track_width / 2 - 2.0
  #elif track_curvature < 0.0:
    #track_distance += track_width / 2 - 2.0
  lane_error = atan2( track_distance, 20 )
  return -yaw_error - lane_error

def braking_distance( speed, desired_speed, mu = 0.3 ):
  if speed < desired_speed:
    return 0
  else:
    return ( speed ** 2 - desired_speed ** 2 ) / ( 2.0 * mu * 9.81 ) + 2.0

def max_speed( curvature, mu = 0.3 ):
  result = 0
  if curvature == 0:
    result = 1E6
  else:
    result = ( mu * 9.81 / np.abs( curvature ) ) ** 0.5
  return result

def acceleration( 
  speed, desired_speed, 
  gear_ratio, max_rpm, wheel_radius, rpm
) :
  if desired_speed < speed:
    return 0.0

  if speed + 10 < desired_speed:
    return 1.0
    #wheel_diameter = wheel_radius * 2 * pi
    #desired_rpm = ( desired_speed / wheel_diameter ) * gear_ratio
  else:
    desired_rpm = rpm * speed / desired_speed
  
  result = min( desired_rpm / max_rpm, 1.0 )
  return result

def gear( 
speed, 
gear, gear_ratio, lower_gear_ratio, max_rpm, wheel_radius, 
shift = 0.8, shift_margin = 4.0 
):
  if gear <= 0:
    return 1
  omega = max_rpm / gear_ratio
  if omega * wheel_radius * shift < speed:
    return gear + 1
  if lower_gear_ratio != 0.0:
    omega = max_rpm / lower_gear_ratio
    if gear > 1.0 and omega * wheel_radius * shift > speed + shift_margin:
      return gear - 1
  return gear

def brake():
  return 0.0
