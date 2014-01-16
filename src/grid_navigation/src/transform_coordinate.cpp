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


//oc means obstacle with respect to the car.
//oce means obstacle with respect to centre of the curvature.




Mat transform_trackspace(double x_oc, double y_oc, int id){
	
	
	
	
	Mat Rt 	 = (Mat_<double>(2,2)<<cos(-ry),-sin(-ry),sin(-ry),cos(-ry));
	Mat X_oc = (Mat_<double>(2,1)<<x_oc,y_oc);
	Mat Ld 	 = (Mat_<double>(2,1)<<0, -yo);
	
	Mat X_ot(2,1,CV_64FC1,Scalar(0));
	
	X_ot = Rt*X_oc + Ld; //r1
	
	double arclength;
	double ytrack;
	double alpha;

	
	if(ch!=0){
	Mat X_cet = (Mat_<double>(2,1)<<0,-fabs((1.0/ch))); //r2
	Mat X_oce = X_ot-X_cet; //r3
	
	alpha = atan2( X_oce.at<double>( 0, 0 ), X_oce.at<double>( 1, 0 ) );
	
		arclength = alpha*fabs(((1.0/ch)));  //required xco-ordinate
		ytrack =  X_ot.at<double>(1,0);
		//cout<<"curvature "<<ch<<endl;
	}
	else
		{
		arclength = X_ot.at<double>(0,0);
		ytrack = X_ot.at<double>(1,0);
		}

	Mat points = (Mat_<double>(1,2)<<arclength,ytrack);
	return points;
	
}










void Callback_road(const e_motion_perception_msgs::Lane::ConstPtr& msg)
{
wd = msg->width;
ch = msg->ch;
cl = msg->cl;
yo = msg->y0;
ry = msg->relative_yaw; 
}







void Callback_Obstacle(const car_navigation_msgs::Obstacles& obstacles){
	
	vector<Point2f> t_points;
	car_navigation_msgs::Obstacles obs;
	
	int i;

	
	for(i= 0;i< obstacles.obstacles.size(); i++){
	
	Mat points = transform_trackspace(obstacles.obstacles[i].pose.x, obstacles.obstacles[i].pose.y,obstacles.obstacles[i].id);
	
	car_navigation_msgs::Obstacle O;
	O.id = obstacles.obstacles[i].id;
	O.pose.x = points.at<double>(0,0);
	O.pose.y = points.at<double>(0,1);
	O.pose.theta = obstacles.obstacles[i].pose.theta;
	O.width = obstacles.obstacles[i].width;
	O.height = obstacles.obstacles[i].height;
	O.speed = obstacles.obstacles[i].speed;
	obs.obstacles.push_back(O); 
	
	}
	
	car_navigation_msgs::Obstacle own_car;
	own_car.id = -2;
	own_car.pose.x = 0;
	own_car.pose.y = yo;
	own_car.pose.theta = ry;
	own_car.width = 1.9;
	own_car.height = 4.5;
	obs.obstacles.push_back(own_car); 
	///Checking the transformation.
	///ry=0;yo=0;
	///cout<<"width "<<wd<<endl;
	
	//Mat a = transform_trackspace(0,wd/2,-1);
	Mat a = (Mat_<double>(1,2)<<0,wd/2);
	
	car_navigation_msgs::Obstacle To1;
	To1.id  = -1;
	To1.pose.x = a.at<double>(0,0);
	To1.pose.y = a.at<double>(0,1);
	To1.pose.theta = 0;
	To1.width  = 0.4;
	To1.height = 0.4;
	obs.obstacles.push_back(To1); 
	
	//Mat b = transform_trackspace(200,wd/2,-1);
	Mat b = (Mat_<double>(1,2)<<200,wd/2);
	car_navigation_msgs::Obstacle To2;
	To2.id  = -1;
	To2.pose.x = b.at<double>(0,0);
	To2.pose.y = b.at<double>(0,1);
	To2.pose.theta = 0;
	To2.width  = 0.4;
	To2.height = 0.4;
	obs.obstacles.push_back(To2); 
	
	//Mat c = transform_trackspace(200,-wd/2,-1);
	Mat c = (Mat_<double>(1,2)<<200,-wd/2);
	car_navigation_msgs::Obstacle To3;
	To3.id  = -1;
	To3.pose.x = c.at<double>(0,0);
	To3.pose.y = c.at<double>(0,1);
	To3.pose.theta = 0;
	To3.width  = 0.4;
	To3.height = 0.4;
	obs.obstacles.push_back(To3); 
	
	//Mat d = transform_trackspace(0,-wd/2,-1);
	Mat d = (Mat_<double>(1,2)<<0,-wd/2);
	car_navigation_msgs::Obstacle To4;
	To4.id  = -1;
	To4.pose.x = d.at<double>(0,0);
	To4.pose.y = d.at<double>(0,1);
	To4.pose.theta = 0;
	To4.width  = 0.4;
	To4.height = 0.4;
	obs.obstacles.push_back(To4); 
	
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
