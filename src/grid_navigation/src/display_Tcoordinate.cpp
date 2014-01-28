#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <e_motion_perception_msgs/Lane.h>
#include <car_navigation_msgs/Obstacle.h>
#include <car_navigation_msgs/Obstacles.h>
#include <car_navigation_msgs/Status.h>
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
ros::Publisher vis_pub_cl;

visualization_msgs::Marker visualMarker(double x, 
                                                double y, 
                                                double theta, 
                                                double width, 
                                                double height, 
                                                int id, 
                                                string ns, Mat Color){

	std::ostringstream s;
	s<<id;
	string st;
	st = s.str();

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time();
	marker.ns = ns + st;
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = cos( 0.5 * theta );
	marker.pose.orientation.y = sin( 0.5 * theta );
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x =height;
	marker.scale.y =width;
	marker.scale.z =.1;
	marker.color.a = 1.0;
	marker.color.r = Color.at<double>(0,0);
	marker.color.b = Color.at<double>(1,0);
	marker.color.g = Color.at<double>(2,0);
	return marker;
}

void Callback_road(const e_motion_perception_msgs::Lane::ConstPtr& msg){
	vector<visualization_msgs::Marker> markers;
	visualization_msgs::MarkerArray markers_msg;
	Mat ColorCar  = (Mat_<double>(3,1)<<0.0,0.0,1.0);
	Mat ColorLane = (Mat_<double>(3,1)<<0.0,1.0,0.0);

	markers.push_back(visualMarker(0, msg->y0, msg->relative_yaw, 1.9, 4.5, -5,"car_coordinates", ColorCar));
	markers.push_back(visualMarker(0, msg->width/2, 0, 0.6, 0.6, -1, "lane_coordinates", ColorLane));
	markers.push_back(visualMarker(50,msg->width/2, 0, 0.6, 0.6, -2, "lane_coordinates", ColorLane));
	markers.push_back(visualMarker(50,-msg->width/2, 0, 0.6, 0.6,-3, "lane_coordinates", ColorLane));
	markers.push_back(visualMarker(0, -msg->width/2, 0, 0.6, 0.6,-4, "lane_coordinates", ColorLane));
	markers_msg.markers = markers;
	vis_pub_cl.publish(markers_msg);
}

void Callback_display(const car_navigation_msgs::Obstacles& obstacles){	
	
	int count = 0;
	int obstacle_counter = 0;
	vector<geometry_msgs::Point> obstaclerepo;	
	vector<Point3f> t_points;
	vector<visualization_msgs::Marker> markers;
	vector<visualization_msgs::Marker> markersP;
	Mat Color = (Mat_<double>(3,1)<<1.0,0.0,0.0);

	for(int i = 0;i<obstacles.obstacles.size(); i++){
		t_points.push_back(Point3f(obstacles.obstacles[i].pose.x, obstacles.obstacles[i].pose.y, 								obstacles.obstacles[i].pose.theta));	
	} 

	for(vector<Point3f>::iterator it = t_points.begin(); it!= t_points.end(); ++it) {
		markers.push_back(visualMarker((it)->x,(it)->y,(it)->z,obstacles.obstacles[count].width,
						obstacles.obstacles[count].height, count, "track_coordinates",Color));			
	count++;
	}

	visualization_msgs::MarkerArray markers_msg;
	markers_msg.markers = markers;
	vis_pub.publish(markers_msg);
}

// 20140128_yyf
#define TrackWidth 20
#define TrackLengthShow 100
car_navigation_msgs::Status status;

void CallBack_status(const car_navigation_msgs::Status& msg)
{
	status = msg;

	vector<visualization_msgs::Marker> markers;
	visualization_msgs::MarkerArray markers_msg;
	Mat ColorCar  = (Mat_<double>(3,1)<<0.0,0.0,1.0);
	Mat ColorLane = (Mat_<double>(3,1)<<1.0,1.0,1.0);

	markers.push_back(visualMarker(0, status.pose.y, status.pose.theta, 1.9, 4.5, -5,"car_coordinates", ColorCar));
	markers.push_back(visualMarker(0, TrackWidth/2, 0, 1, TrackLengthShow, -1, "lane_coordinates", ColorLane));
	markers.push_back(visualMarker(0, 0, 0, 1, TrackLengthShow, -2, "lane_coordinates", ColorLane));
	markers.push_back(visualMarker(0, -TrackWidth/2, 0, 1, TrackLengthShow, -3, "lane_coordinates", ColorLane));
	markers_msg.markers = markers;
	vis_pub_cl.publish(markers_msg);
}

Point3f CoordinateTranslation(const car_navigation_msgs::Obstacle& obstacle)
{
	Point3f p, pResult;
	p.x = obstacle.pose.x;
    p.y = obstacle.pose.y;
	p.z = obstacle.pose.theta;

	float yaw = status.pose.theta;
	float x = 0;
	float y = status.pose.y;
	pResult.x = p.x * cos(yaw) - p.y * sin(yaw);
	pResult.y = p.y * cos(yaw) + p.x * sin(yaw) + y;
	pResult.z = p.z + yaw;

	//printf("%f\t%f\t%f\n",yaw, p.y, pResult.y);

	return pResult;
}

void Callback_obstacles(const car_navigation_msgs::Obstacles& obstacles){	
	
	int count = 0;
	int obstacle_counter = 0;
	vector<geometry_msgs::Point> obstaclerepo;	
	vector<Point3f> t_points;
	vector<visualization_msgs::Marker> markers;
	vector<visualization_msgs::Marker> markersP;
	Mat Color = (Mat_<double>(3,1)<<1.0,0.0,0.0);

	for(int i = 0;i<obstacles.obstacles.size(); i++){
		t_points.push_back(CoordinateTranslation(obstacles.obstacles[i]));	
	} 

	for(vector<Point3f>::iterator it = t_points.begin(); it!= t_points.end(); ++it) {
		markers.push_back(visualMarker((it)->x,(it)->y,(it)->z,obstacles.obstacles[count].width,
						obstacles.obstacles[count].height, count, "track_coordinates",Color));			
	count++;
	}

	visualization_msgs::MarkerArray markers_msg;
	markers_msg.markers = markers;
	vis_pub.publish(markers_msg);
}



int main(int argc, char*argv[]){
	ros::init(argc, argv, "displayTcoodrinate");
	ros::NodeHandle n;
	// ros::Subscriber dp = n.subscribe("/trackcordinates", 1, Callback_display);
	// ros::Subscriber sr = n.subscribe("/road", 1, Callback_road);

	ros::Subscriber obstaclesSub = n.subscribe("/obstacles", 10, Callback_obstacles);
	ros::Subscriber statusSub = n.subscribe("/status", 10, CallBack_status);

	vis_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 1000 );
	vis_pub_cl = n.advertise<visualization_msgs::MarkerArray>( "/visualization_markercarlane", 1000 );
	ros::spin();
 return 0;	
}
