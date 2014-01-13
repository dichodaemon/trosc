#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <e_motion_perception_msgs/Lane.h>
#include <car_navigation_msgs/Obstacles.h>
#include <grid_navigation/TrackTransform.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GridCells.h>
#include <cstring>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#define PI 3.141592


#define RESOLUTION 1
#define WINDOW_SIZE 100








using namespace cv;
using namespace std;

double wd,ch,cl,yo,ry;

ros::Publisher pub;
ros::Publisher vis_pub;
ros::Publisher vis_pubP;
ros::Publisher pos_pub;

ros::Publisher obstaclemap_pub;




visualization_msgs::Marker visualMarker(double x, double y, int id){
 
		std::ostringstream s;
		s<<id;
		string st;
		st = s.str();
 
	    visualization_msgs::Marker marker;
		marker.header.frame_id = "/my_frame";
		marker.header.stamp = ros::Time();
		marker.ns = "track_coordinates" + st;
		marker.id = id;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x =.2;
		marker.scale.y =.2;
		marker.scale.z =.2;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.b = 1.0;
		marker.color.g = 1.0;
		return marker;
		//vis_pub.publish(marker);
}


visualization_msgs::Marker visualMarkerPoint(double x, double y, int id){
 
		std::ostringstream s;
		s<<id;
		string st;
		st = s.str();
 
	    visualization_msgs::Marker marker;
		marker.header.frame_id = "/my_frame";
		marker.header.stamp = ros::Time();
		marker.ns = "point_coordinates" + st;
		marker.id = id;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x =.5;
		marker.scale.y =.5;
		marker.scale.z =.5;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.b = 0.0;
		marker.color.g = 0.0;
		return marker;
		//vis_pub.publish(marker);
}



geometry_msgs::PoseStamped PoseArrow(double x, double y, double theta){
 
 
geometry_msgs::PoseStamped carpose;
carpose.header.stamp = ros::Time::now();
carpose.header.frame_id = "/my_frame";
carpose.pose.position.x = x;
carpose.pose.position.y = y;
carpose.pose.position.z = 0;
carpose.pose.orientation.x = 0.0;
carpose.pose.orientation.y = theta;
carpose.pose.orientation.z = 0.0;
carpose.pose.orientation.w = 0.0;

		return carpose;
		//vis_pub.publish(marker);
}




Mat transform_trackspace(double x_oc, double y_oc, int id){
	Mat Rt 	 = (Mat_<double>(2,2)<<cos(-ry),-sin(-ry),sin(-ry),cos(-ry));
	
	Mat X_oc = (Mat_<double>(2,1)<<x_oc,y_oc);
	Mat Ld 	 = (Mat_<double>(2,1)<<0,-yo);
	
	Mat X_ot(2,1,CV_64FC1,Scalar(0));
	
	X_ot = Rt*X_oc + Ld; //r1
	
	double arclength;
	double ytrack;
	double alpha;
	double beta;
	double theta;
	
	if(ch!=0){
	Mat X_cet = (Mat_<double>(2,1)<<0,-((1.0/ch))); //r2
	Mat X_oce = X_ot-X_cet; //r3
	double num   = -X_cet.dot(X_oce);
	double normr2 = norm(X_cet,NORM_L2);
	double normr3 = norm(X_oce,NORM_L2);
	alpha = acos(num/(normr2*normr3));
	
	
	
		arclength = alpha*(1.0/ch);  //required xco-ordinate
		ytrack =  X_ot.at<double>(1,0);
		
		
	
		
	
	}

	{
		arclength = X_ot.at<double>(0,0);
		ytrack = X_ot.at<double>(1,0);
	}
	
	
	
	Mat points = (Mat_<double>(1,2)<<arclength,ytrack);
	//cout<<points<<endl;
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
	vector<visualization_msgs::Marker> markers;
	vector<visualization_msgs::Marker> markersP;
	
	
	
	
	// create a grid cell message for obstacles
  nav_msgs::GridCells gco;
  gco.header.frame_id = "/my_frame";
  gco.header.stamp = ros::Time();
  gco.cell_width = RESOLUTION;
  gco.cell_height = RESOLUTION;

	
	for(int i= 0;i< obstacles.obstacles.size(); i++){
	
	Mat points = transform_trackspace(obstacles.obstacles[i].pose.x, obstacles.obstacles[i].pose.y,obstacles.obstacles[i].id);
	t_points.push_back(Point2f(points.at<double>(0,0),points.at<double>(0,1)));
	//t_points.push_back(Point2f(obstacles.obstacles[i].pose.x,obstacles.obstacles[i].pose.y));
	}
	
	
	int count = 0;
	int obstacle_counter = 0;
	
	vector<geometry_msgs::Point> obstaclerepo;
	for(vector<Point2f>::iterator it = t_points.begin(); it!= t_points.end(); ++it) {
		
		markers.push_back(visualMarker((it)->x,(it)->y,count));
		count++;
		
		
		{
			
			geometry_msgs::Point a;
			a.x = ((it)->x)* RESOLUTION;
			a.y = ((it)->y)* RESOLUTION;
			a.z = 0.0;
			obstaclerepo.push_back(a);
		}
		
	}
	
 gco.cells.resize(obstaclerepo.size());
 while (!obstaclerepo.empty())
 {
    gco.cells[obstacle_counter++] = obstaclerepo.back();
    obstaclerepo.pop_back();
  }
	 
	
	
	
	
    
    visualization_msgs::MarkerArray markers_msg;
    markers_msg.markers = markers;
    vis_pub.publish(markers_msg);
    obstaclemap_pub.publish(gco);
	
	///Checking the transformation.
	///ry=0;yo=0;
	Mat a = transform_trackspace(0,wd/2,-1);
	Mat b = transform_trackspace(200,wd/2,-1);
	Mat c = transform_trackspace(200,-wd/2,-1);
	Mat d = transform_trackspace(0,-wd/2,-1);
	
	markersP.push_back(visualMarkerPoint(a.at<double>(0,0),a.at<double>(0,1),0));
	markersP.push_back(visualMarkerPoint(b.at<double>(0,0),b.at<double>(0,1),1));
	markersP.push_back(visualMarkerPoint(c.at<double>(0,0),c.at<double>(0,1),2));
	markersP.push_back(visualMarkerPoint(d.at<double>(0,0),d.at<double>(0,1),3));
	
	
	//markersP.push_back(visualMarkerArrow(0,0,ry));
	
    visualization_msgs::MarkerArray markers_msgP;
    markers_msgP.markers = markersP;
    vis_pubP.publish(markers_msgP);
	
	pos_pub.publish(PoseArrow(0,0,ry));
	

	
	
}





int main(int argc, char*argv[]){

  ros::init(argc, argv, "listener_race_messages");
  ros::NodeHandle nh;
  ros::Subscriber subroad = nh.subscribe("/road", 1, Callback_road);
  ros::Subscriber subobst = nh.subscribe("/obstacles", 1000, Callback_Obstacle);
  pub = nh.advertise<grid_navigation::TrackTransform>("/trackcordinates",1000);
  vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 1000 );
  vis_pubP = nh.advertise<visualization_msgs::MarkerArray>( "/visualization_markerP", 1000 );
  pos_pub = nh.advertise<geometry_msgs::PoseStamped>( "/pose", 1000 );
  
  obstaclemap_pub = nh.advertise<nav_msgs::GridCells>("/occupancygrid", 1000);
  ros::spin();


return 0;	
	

}























//if(xp > 0 && xp < SIZE && yp > 0 && yp < SIZE)

/////////////Rough Work


//ROS_INFO("!!!!!!!");
/*grid_navigation::TrackTransform msg;
	msg.id = id;
	msg.x = arclength;
	msg.y = ytrack;
	
	
	pub.publish(msg);
	
	
	visualMarker(arclength, ytrack);*/
	
	//cout<<"[x, y] "<<"["<<arclength<<", "<<ytrack<<" ]"<<endl;
	
	
	
	
	//cout<<obstacles.obstacles[i].id<<endl;
	//cout<<obstacles.obstacles[i].pose.x<<endl;
	//cout<<obstacles.obstacles[i].pose.y<<endl;
	//cout<<obstacles.obstacles[i].pose.theta<<endl;
//std::cout<<"Width: "<<wd<<" Road Curvature: "<<ch<<" Variation of road curvature: "<<cl<<" lateral displacement: "<<yo<<" relative yaw: "<< ry << std::endl;

//http://cs225turtle.googlecode.com/svn/trunk/project2/local_obstacles/src/local_obstacles.cpp

//yo is the lateral displacement 
//ch is road curvature from the center of the lane
//ry is the relative yaw
//wd is the width of the lane
	//cout<<obstacles.obstacles.size()<<endl;
	
	//beta = atan(sqrt((denom*denom) - (num*num))/num);
	//cout<<beta*(180/PI)<<endl;





  /*visualization_msgs::MarkerArray markers_msgP;
    markers_msgP.markers = markersP;
    vis_pubP.publish(markers_msgP);
	visualization_msgs::Marker visualMarkerPoint(double x, double y, int id){
 
		std::ostringstream s;
		s<<id;
		string st;
		st = s.str();
 
	    visualization_msgs::Marker marker;
		marker.header.frame_id = "/my_frame";
		marker.header.stamp = ros::Time();
		marker.ns = "point_coordinates" + st;
		marker.id = id;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x =.5;
		marker.scale.y =.5;
		marker.scale.z =.5;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.b = 0.0;
		marker.color.g = 0.0;
		return marker;
		//vis_pub.publish(marker);
}*/


/*else{
			markersP.push_back(visualMarkerPoint(obstacles.obstacles[i].pose.x,obstacles.obstacles[i].pose.y,obstacles.obstacles[i].id));
		}*/	
		
		
//double num   = -X_cet.dot(X_oce);
	/*double normr2 = norm(X_cet,NORM_L2);
	double normr3 = norm(X_oce,NORM_L2);
	alpha = acos(num/(normr2*normr3));*/
