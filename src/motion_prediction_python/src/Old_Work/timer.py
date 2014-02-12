#!/usr/bin/env python

import rospy

def callback1(event):
  print 'In call back1'

def callback2(event):
  print 'In call back2'
  
def listener():
  rospy.init_node('listener', anonymous = True)
  rospy.Timer(rospy.Duration(.1), callback1)
  rospy.Timer(rospy.Duration(1), callback2)
  rospy.spin()

if __name__ == '__main__':
  listener()
