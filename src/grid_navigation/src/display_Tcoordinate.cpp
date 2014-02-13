#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <e_motion_perception_msgs/Lane.h>
#include <car_navigation_msgs/Obstacle.h>
#include <car_navigation_msgs/Obstacles.h>
#include <car_navigation_msgs/Status.h>
#include <car_navigation_msgs/BufferData.h>
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
#include <algorithm>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#define PI 3.141592


#define RESOLUTION 1
#define WINDOW_SIZE 100
using namespace cv;
using namespace std;
using namespace message_filters;
ros::Publisher obstaclemap_pub;
ros::Publisher vis_pub;
ros::Publisher vis_pub_cl;

const float zScale_Car = 2;

// Add Marker is 0, remove marker is 2
visualization_msgs::Marker visualMarker(double x, 
                                                double y, 
                                                double theta, 
                                                double width, 
                                                double height, 
                                                int id, 
                                                string ns, Mat Color,
												int AddOrRemove = 0,
												float zScale = 0.1){

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
	marker.action = AddOrRemove;//visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = zScale/2;
	marker.pose.orientation.x = cos( 0.5 * theta );
	marker.pose.orientation.y = sin( 0.5 * theta );
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = height;
	marker.scale.y = width;
	marker.scale.z = zScale;
	marker.color.a = 1.0;
	marker.color.r = Color.at<double>(0,0);
	marker.color.b = Color.at<double>(1,0);
	marker.color.g = Color.at<double>(2,0);
	return marker;
}

/*
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
*/
// 20140128_yyf
//#include <car_navigation_msgs/BufferData.h>

const float TrackWidth = 20;
const float LaneWidth = 0.5;
const float TrackLengthShow = 100;
const float CenterLineLength = 10;		// length of one center line (white+blank)
const float CenterLineFill = 5;		// length of one white line
car_navigation_msgs::Status status;
float length = 0;
int MinFlag;

void Callback_status(const car_navigation_msgs::Status& msg)
{
	if (abs(msg.pose.x - status.pose.x)<100)
		length = length + msg.pose.x - status.pose.x;
	
	status = msg;

	vector<visualization_msgs::Marker> markers;
	visualization_msgs::MarkerArray markers_msg;
	Mat ColorCar  = (Mat_<double>(3,1)<<0.0,0.0,1.0);
	Mat ColorLane = (Mat_<double>(3,1)<<1.0,1.0,1.0);
	markers.push_back(visualMarker(0, status.pose.y, status.pose.theta, 1.9, 4.5, -5,"car_coordinates", ColorCar, 0, zScale_Car));

	markers.push_back(visualMarker(0, TrackWidth/2.0, 0, LaneWidth, TrackLengthShow, -1, "lane_coordinates", ColorLane));
	markers.push_back(visualMarker(0, -TrackWidth/2.0, 0, LaneWidth, TrackLengthShow, -2, "lane_coordinates", ColorLane));

	// draw center line
	float relativePose = length - int(length/CenterLineLength)*CenterLineLength;
	float nowPose = -TrackLengthShow/2.0 - relativePose;
	float left,right;

	while (nowPose+CenterLineFill/2.0 < -TrackLengthShow/2.0)
		nowPose += CenterLineLength;
	int flag = -3;
	while (nowPose-CenterLineFill/2.0 < TrackLengthShow/2.0)
	{
		left = std::max(-TrackLengthShow/2.0, nowPose-CenterLineFill/2.0);
		right = std::min(TrackLengthShow/2.0, nowPose+CenterLineFill/2.0);
		markers.push_back(visualMarker( (left+right)/2.0, 0, 0, LaneWidth, (right-left), flag--, "lane_coordinates", ColorLane));
		nowPose += CenterLineLength;
	}
	
	// delete non-existed lane segments
	if (MinFlag<flag)
	{
		for (int i=MinFlag+1; i<=flag; i++)
			markers.push_back(visualMarker( (left+right)/2.0, 0, 0, LaneWidth, (right-left), i, "lane_coordinates", ColorLane,2));
	}
	MinFlag = flag;

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
	float x = status.pose.x;
	float y = status.pose.y;
	//pResult.x = p.x * cos(yaw) - p.y * sin(yaw);
	//pResult.y = p.y * cos(yaw) + p.x * sin(yaw) + y;
	//pResult.z = p.z + yaw;
	pResult.x = p.x - x;
	pResult.y = p.y;
	pResult.z = p.z;

	//printf("%f\t%f\t%f\n",yaw, p.y, pResult.y);

	return pResult;
}

void Callback_obstacles(const car_navigation_msgs::Obstacles& obstacles){	
	
	int count = 0;
	int obstacle_counter = 0;
	vector<geometry_msgs::Point> obstaclerepo;	
	vector<Point3f> t_points;
	vector<visualization_msgs::Marker> markers;
	Mat Color = (Mat_<double>(3,1)<<1.0,0.0,0.0);

	for(int i = 0;i<obstacles.obstacles.size(); i++){
		t_points.push_back(CoordinateTranslation(obstacles.obstacles[i]));	
	} 

	for(vector<Point3f>::iterator it = t_points.begin(); it!= t_points.end(); ++it) {
		markers.push_back(visualMarker((it)->x,(it)->y,(it)->z,obstacles.obstacles[count].width, obstacles.obstacles[count].height, count, "track_coordinates",Color, 0, zScale_Car));			
	count++;
	}

	visualization_msgs::MarkerArray markers_msg;
	markers_msg.markers = markers;
	vis_pub.publish(markers_msg);
}

void Callback_status_obstacles(const car_navigation_msgs::Status& msg, const car_navigation_msgs::Obstacles& obstacles)
{
	Callback_status(msg);
	Callback_obstacles(obstacles);
}

void Callback_buffer(const car_navigation_msgs::BufferData& buf)
{
	Callback_status(buf.status);
	Callback_obstacles(buf.obstacles);
}



int main(int argc, char*argv[]){
	ros::init(argc, argv, "displayTcoodrinate");
	ros::NodeHandle n;
	// ros::Subscriber dp = n.subscribe("/trackcordinates", 1, Callback_display);
	// ros::Subscriber sr = n.subscribe("/road", 1, Callback_road);

	//ros::Subscriber statusSub = n.subscribe("/status", 1, Callback_status);
	//ros::Subscriber obstaclesSub = n.subscribe("/obstacles", 1, Callback_obstacles);
	ros::Subscriber bufferSub = n.subscribe("/buffer_data", 10, Callback_buffer);
	
	// sync two data
	//message_filters::Subscriber<car_navigation_msgs::Status> statusSub(n, "/status", 1);
	//message_filters::Subscriber<car_navigation_msgs::Obstacles> obstaclesSub(n, "/obstacles", 1);
	//TimeSynchronizer <car_navigation_msgs::Status, car_navigation_msgs::Obstacles> sync(statusSub, obstaclesSub, 10);
	//sync.registerCallback(Callback_status_obstacles);
	
	vis_pub = n.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 1000 );
	vis_pub_cl = n.advertise<visualization_msgs::MarkerArray>( "/visualization_markercarlane", 1000 );
	ros::spin();
 return 0;	
}

