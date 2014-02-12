#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <e_motion_perception_msgs/Lane.h>
#include <car_navigation_msgs/Obstacles.h>
#include <kalman_prediction_msg/ObstaclesPrediction.h>
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

using namespace cv;
using namespace std;

ros::Publisher prediction_pub;
ros::Publisher visualpred_pub;


//xpred, ypred, 


visualization_msgs::Marker visualMarker(double xpred, 
                                        double ypred, 
                                        double theta_major, 
                                        double eigx, 
                                        double eigy,
                                        int type, 
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
 marker.type = type;
 marker.action = visualization_msgs::Marker::ADD;
 marker.pose.position.x = xpred;
 marker.pose.position.y = ypred;
 marker.pose.position.z = 0;
 marker.pose.orientation.x = cos( 0.5 * (theta_major ) ) ;
 marker.pose.orientation.y = sin( 0.5 * (theta_major ) );
 marker.pose.orientation.z = 0.0;
 marker.pose.orientation.w = 0.0;
 marker.scale.x = fabs(eigx)+2;
 marker.scale.y = fabs(eigy)+2;
 marker.scale.z =.1;
 marker.color.a = 1.0;
 marker.color.r = Color.at<double>(0,0);
 marker.color.b = Color.at<double>(1,0);
 marker.color.g = Color.at<double>(2,0);
 return marker;
}









void Callback_prediction(const kalman_prediction_msg::ObstaclesPrediction& obstaclesprediction){
	
	int count1 = 0;
	int count2 = 0;
	vector<visualization_msgs::Marker> markers;
	
	Mat ColorObstacles = (Mat_<double>(3,1)<<1.0,0.0,0.0);
	Mat ColorEllipse = (Mat_<double>(3,1)<<0.0,1.0,0.0);
	
	
	for(int i= 0;i< obstaclesprediction.obstaclesprediction.size(); i++){
	double x = obstaclesprediction.obstaclesprediction[i].pose.x;
	double y = obstaclesprediction.obstaclesprediction[i].pose.y;
	double t = obstaclesprediction.obstaclesprediction[i].pose.theta; //this theta is vy/vx
  double eigx = obstaclesprediction.obstaclesprediction[i].eigx;
  double eigy = obstaclesprediction.obstaclesprediction[i].eigy;
  
  
	//cout<<x<<", "<<y<<", "<<t<<" , "<<eigx<<" , "<<eigy<<endl;
  //markers.push_back(visualMarker(x, y, t, eigx*4, eigy*4, 3, count1++, "obstacle_coordinates",ColorObstacles));
	 markers.push_back(visualMarker(x, y, t, eigx, eigy, 3, count2++, "obstacle_uncertainity",ColorEllipse));
	}
	
 visualization_msgs::MarkerArray markers_msg;
 markers_msg.markers = markers;
 visualpred_pub.publish(markers_msg);
}


int main(int argc, char*argv[]){
 ros::init(argc, argv, "displayPcoodrinate");
 ros::NodeHandle n;
 ros::Subscriber sb = n.subscribe("/prediction",1, Callback_prediction); ///Change the message topic as per requirements.
 visualpred_pub = n.advertise<visualization_msgs::MarkerArray>( "/ellipse_marker", 1000 );
 
 ros::spin();
 return 0;	
}

