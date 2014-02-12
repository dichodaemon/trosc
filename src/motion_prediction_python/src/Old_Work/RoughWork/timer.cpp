#include "ros/ros.h"
#include <iostream>

/**
 * This tutorial demonstrates the use of timer callbacks.
 */
using namespace std;


void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
  cout<<"Timer1"<<endl;
  
}

void callback2(const ros::TimerEvent&)
{
  ROS_INFO("Callback 2 triggered");
  cout<<"Timer2"<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**
   * Timers allow you to get a callback at a specified rate.  Here we create
   * two timers at different rates as a demonstration.
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);

  ros::spin();

  return 0;
}
