#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <e_motion_perception_msgs/Lane.h>
#include <car_navigation_msgs/Obstacles.h>
#include <cstring>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

using namespace cv;
using namespace std;

double wd,ch,cl,yo,ry;
ros::Publisher pub;

/**oc means obstacle with respect to the car.
oce means obstacle with respect to centre of the curvature.**/



Mat transform_trackspace(double x_oc, double y_oc, float theta, int id ) {
		
 double arclength;
 double ytrack;
 double alpha;
 double yaw;
 	
 Mat Rt   = (Mat_<double>( 2, 2 ) << cos(ry),-sin(ry),sin(ry),cos(ry));
 Mat X_oc = (Mat_<double>( 2, 1 ) << x_oc,y_oc);
 Mat Ld   = (Mat_<double>( 2, 1 ) << 0, yo);
 
 Mat X_ot(2,1,CV_64FC1,Scalar(0));
 X_ot = Rt*X_oc + Ld; 
	
 if( ch > 0 ){ 	 
  Mat X_cet = (Mat_<double>(2,1) << 0, 1.0 / ch); 
  Mat X_oce = X_ot - X_cet; 
  alpha = atan2( X_oce.at<double>( 0, 0 ), -X_oce.at<double>( 1, 0 ) );
  yaw = theta + ry - alpha;	
  arclength = alpha / ch;  
  ytrack =  X_ot.at<double>(1,0);		 
  } else if ( ch < 0 ) {  
  Mat X_cet = (Mat_<double>(2,1) << 0, 1.0 / ch); 
  Mat X_oce = X_ot - X_cet; 
  alpha = atan2( X_oce.at<double>( 0, 0 ), X_oce.at<double>( 1, 0 ) );
  yaw = theta + ry + alpha;	
  arclength = alpha / fabs(ch);  
  ytrack =  X_ot.at<double>(1,0);
  } else {  
  arclength = X_ot.at<double>(0,0);
  ytrack = X_ot.at<double>(1,0);
  yaw = theta + ry;
  } 
  ///Transformed x,y,theta
 Mat points = (Mat_<double>(1,3)<<arclength,ytrack, yaw);
 return points;
}

void Callback_road(const e_motion_perception_msgs::Lane::ConstPtr& msg){
 wd = msg->width;
 ch = msg->ch;
 cl = msg->cl;
 yo = msg->y0;
 ry = msg->relative_yaw; 
}

void Callback_Obstacle(const car_navigation_msgs::Obstacles& obstacles) {
	
 vector<Point2f> t_points;
 car_navigation_msgs::Obstacles obs;

 for(int i= 0;i< obstacles.obstacles.size(); i++){
  Mat points = transform_trackspace(obstacles.obstacles[i].pose.x, obstacles.obstacles[i].pose.y,obstacles.obstacles[i].pose.theta, obstacles.obstacles[i].id);
  car_navigation_msgs::Obstacle O;
  O.id = obstacles.obstacles[i].id;
  O.pose.x = points.at<double>(0,0);
  O.pose.y = points.at<double>(0,1);
  O.pose.theta = points.at<double>(0, 2);
  O.width = obstacles.obstacles[i].width;
  O.height = obstacles.obstacles[i].height;
  O.speed = obstacles.obstacles[i].speed;
  obs.obstacles.push_back(O); 
 }
 pub.publish(obs);
}

int main(int argc, char*argv[]){
 ros::init(argc, argv, "transform");
 ros::NodeHandle nh;
 ros::Subscriber subroad = nh.subscribe("/road", 1, Callback_road);
 ros::Subscriber subobst = nh.subscribe("/obstacles", 1000, Callback_Obstacle);
 pub = nh.advertise<car_navigation_msgs::Obstacles>("/trackcordinates",1000);
 ros::spin();
 return 0;	
}
