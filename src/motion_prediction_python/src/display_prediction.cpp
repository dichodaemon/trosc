#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <e_motion_perception_msgs/Lane.h>
#include <car_navigation_msgs/Obstacles.h>
#include <kalman_prediction_msg/Predictions.h>
#include <kalman_prediction_msg/Prediction.h>
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
#define TS 20

double offset = 0;
double offset_step = 100;

using namespace cv;
using namespace std;

ros::Publisher prediction_pub;
ros::Publisher visualpred_pub;



const float TrackWidth = 20;
const float LaneWidth = 0.5;
const float TrackLengthShow = 200;
const float CenterLineLength = 10;
const float CenterLineFill = 5;
float length = 0;
int MinFlag;




//xpred, ypred, 


visualization_msgs::Marker visualMarker(double xpred, 
                                        double ypred, 
                                        double theta_major, 
                                        double eigx, 
                                        double eigy,
                                        int type, 
                                        int id, 
                                        string ns, Mat Color,
                                        int AddOrRemove = 0){

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
 marker.action = AddOrRemove;//visualization_msgs::Marker::ADD;
 marker.pose.position.x = xpred;
 marker.pose.position.y = ypred;
 marker.pose.position.z = 0;
 marker.pose.orientation.x = cos( 0.5 * (theta_major ) ) ;
 marker.pose.orientation.y = sin( 0.5 * (theta_major ) );
 marker.pose.orientation.z = 0.0;
 marker.pose.orientation.w = 0.0;
 marker.scale.x = fabs(eigx);
 marker.scale.y = fabs(eigy);
 marker.scale.z = 0.1;
 
 marker.color.r = Color.at<double>(0,0);
 marker.color.b = Color.at<double>(1,0);
 marker.color.g = Color.at<double>(2,0);
 marker.color.a = .4;
 
 
 return marker;
}









void Callback_prediction(const kalman_prediction_msg::Predictions& prediction){
	
	int count1 = 0;
	int count2 = 0;
  vector<visualization_msgs::Marker> markers;
	
	
	Mat ColorObstacles = (Mat_<double>(3,1)<<1.0,0.0,0.0);
	Mat ColorEllipse;// = (Mat_<double>(3,1)<<0.0,1.0,0.0);
  Mat ColorCar = (Mat_<double>(3,1)<<0.0,0.0,1.0);
  Mat ColorLane = (Mat_<double>(3,1)<<1.0,1.0,1.0);
	
	Mat Mean, Cov, Eig;
  int x,y,t;
  
	for(int i= 0;i< prediction.prediction.size(); i++){
  //cout<<"ID = "<<prediction.prediction[i].id<<" ";
  
    for(int j= 0; j<TS; j++){
      const kalman_prediction_msg::PredictionOneStep& predictiononestep =  prediction.prediction[i].predictiononestep[j]; 
      
      if(j==0){
        Mean = (Mat_<double>(1,4)<<predictiononestep.mean[0],predictiononestep.mean[1],predictiononestep.mean[2],predictiononestep.mean[3]);
        Cov  = (Mat_<double>(2,2)<<predictiononestep.cov[0],predictiononestep.cov[1], predictiononestep.cov[4], predictiononestep.cov[5]);
        eigen(Cov,Eig);
        x = Mean.at<double>(0,0);
        if (i==0 && abs(x-offset)>offset_step)
        {
           offset = (int(x/offset_step))*offset_step;
           //printf("%lf\t%f\n", x, offset);
           x = x - offset;
       }
        y = Mean.at<double>(0,1);
        t = Mean.at<double>(0,3)/Mean.at<double>(0,2);
        markers.push_back(visualMarker(x, y, t, -1.0, 0.0, 1, count1++, "obstacle_coordinates",ColorCar));
        markers.push_back(visualMarker(x, y, t, Eig.at<double>(0,0)*50, Eig.at<double>(0,1)*50, 3, count1++, "obstacle_coordinates",ColorObstacles));
      }
      
      else{
        Mean = (Mat_<double>(1,4)<<predictiononestep.mean[0],predictiononestep.mean[1],predictiononestep.mean[2],predictiononestep.mean[3]);
        Cov  = (Mat_<double>(2,2)<<predictiononestep.cov[0],predictiononestep.cov[1], predictiononestep.cov[4], predictiononestep.cov[5]);
        eigen(Cov,Eig);
        x = Mean.at<double>(0,0);
        x = x - offset; 
        y = Mean.at<double>(0,1);
        t = Mean.at<double>(0,3)/Mean.at<double>(0,2);
        ColorEllipse = (Mat_<double>(3,1)<<0.0,(1.0/TS)*j, 0.0);
        markers.push_back(visualMarker(x, y, t, Eig.at<double>(0,0)*50, Eig.at<double>(0,1)*50, 3, count1++, "obstacle_coordinates",ColorEllipse));
      }
    }
	}
  markers.push_back(visualMarker( 0, TrackWidth/2.0, PI/2, LaneWidth, TrackLengthShow, 1, -1, "lane_coordinates", ColorLane));
	markers.push_back(visualMarker( 0, -TrackWidth/2.0, PI/2, LaneWidth, TrackLengthShow, 1,  -2, "lane_coordinates", ColorLane));


 visualization_msgs::MarkerArray markers_msg;
 markers_msg.markers = markers;
 visualpred_pub.publish(markers_msg);

}


int main(int argc, char*argv[]){
 ros::init(argc, argv, "displayPcoodrinate");
 ros::NodeHandle n;
 ros::Subscriber sb = n.subscribe("/prediction_new",1, Callback_prediction); ///Change the message topic as per requirements.
 visualpred_pub = n.advertise<visualization_msgs::MarkerArray>( "/ellipse_marker", 1000 );
 
 ros::spin();
 return 0;
}

