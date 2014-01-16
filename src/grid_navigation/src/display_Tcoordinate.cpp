#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <e_motion_perception_msgs/Lane.h>
#include <car_navigation_msgs/Obstacles.h>
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
ros::Publisher obstaclemap_pub;
ros::Publisher vis_pub;




visualization_msgs::Marker visualMarker(double x, double y, double theta, double width, double height, int id){
 
		std::ostringstream s;
		s<<id;
		string st;
		st = s.str();
 
	    visualization_msgs::Marker marker;
		marker.header.frame_id = "/my_frame";
		marker.header.stamp = ros::Time();
		marker.ns = "track_coordinates" + st;
		marker.id = id;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = theta;
		marker.pose.orientation.w = 1.0;
		
		
		{
		marker.scale.x =height;
		marker.scale.y =width;
		marker.scale.z =.1;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.b = 0.0;
		marker.color.g = 0.0;
		}
		
		
	
		return marker;
		
}





void Callback_display(const car_navigation_msgs::Obstacles& obstacles){
	
	vector<Point3f> t_points;
	vector<visualization_msgs::Marker> markers;
	vector<visualization_msgs::Marker> markersP;
	
  nav_msgs::GridCells gco;
  gco.header.frame_id = "/my_frame";
  gco.header.stamp = ros::Time();
  gco.cell_width = RESOLUTION;
  gco.cell_height = RESOLUTION;
	
	for(int i = 0;i<obstacles.obstacles.size(); i++){
		t_points.push_back(Point3f(obstacles.obstacles[i].pose.x, obstacles.obstacles[i].pose.y, obstacles.obstacles[i].pose.theta));	
	}
	
	
	int count = 0;
	int obstacle_counter = 0;
	
	vector<geometry_msgs::Point> obstaclerepo;
	for(vector<Point3f>::iterator it = t_points.begin(); it!= t_points.end(); ++it) {
		
		markers.push_back(visualMarker((it)->x,(it)->y,(it)->z,obstacles.obstacles[count].width,obstacles.obstacles[count].height, count));
		
		
			if(obstacles.obstacles[count].id ==-1)
			{
			geometry_msgs::Point a;
			a.x = ((it)->x)* RESOLUTION;
			a.y = ((it)->y)* RESOLUTION;
			a.z = 0.0;
			obstaclerepo.push_back(a);
			}
		count++;
	}
	
	gco.cells.resize(obstaclerepo.size());
	
	while (!obstaclerepo.empty()){
    gco.cells[obstacle_counter++] = obstaclerepo.back();
    obstaclerepo.pop_back();
	}
	 
	
	visualization_msgs::MarkerArray markers_msg;
    markers_msg.markers = markers;
    vis_pub.publish(markers_msg);
    obstaclemap_pub.publish(gco);
    
    
   
    
  
	
}



int main(int argc, char*argv[]){

  ros::init(argc, argv, "displayTcoodrinate");
  ros::NodeHandle n;
  ros::Subscriber dp = n.subscribe("/trackcordinates", 1, Callback_display);
  
  vis_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 1000 );
  
  
  obstaclemap_pub = n.advertise<nav_msgs::GridCells>("/occupancygrid", 1000);
  ros::spin();


return 0;	
	

}
