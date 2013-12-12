import numpy as np
from math import *
import sys

def steering( 
  track_yaw, track_curvature, track_distance, track_width, 
  next_curvature, next_distance
):
  yaw_error = track_yaw 
  #if next_curvature > 0.0:
    #track_distance -= track_width / 2 - 2.0
  #elif next_curvature < 0.0:
    #track_distance += track_width / 2 - 2.0
  #lane_error = atan2( track_distance, next_distance )
  if track_curvature > 0.0:
    track_distance -= track_width / 2 - 2.0
  elif track_curvature < 0.0:
    track_distance += track_width / 2 - 2.0
  lane_error = atan2( track_distance, 20 )
  return -yaw_error - lane_error

def max_speed( curvature, track_yaw, track_distance, mu = 0.3 ):
  result = 0
  if curvature == 0:
    result = 1E6
  else:
    result = ( mu * 9.81 / np.abs( curvature ) ) ** 0.5
  return result

def acceleration( 
  speed, desired_speed, 
  gear_ratio, max_rpm, wheel_radius,
  acceleration_margin = 1.0
) :
  if desired_speed > speed + acceleration_margin:
    return 1.0
  else:
    return desired_speed / wheel_radius * gear_ratio / max_rpm

def gear( 
  speed, 
  gear, gear_ratio, lower_gear_ratio, max_rpm, wheel_radius, 
  shift = 0.9, shift_margin = 4.0 
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
