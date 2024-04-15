#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

class gps_to_odom {

private:
  ros::NodeHandle n;
  ros::Publisher gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1);
  ros::Subscriber gps_sub;

  double lat_zero;
  double lon_zero;
  double alt_zero;

  double a = 6378137; // Semi-major axis of earth given in meters
  double b = 6356752.3142; // Semi-minor axis of earth given in meters
  double e = 0.0818191908426; // Eccentricity of earth

  double N_prev = 0;
  double E_prev = 0;
  double D_prev = 0;

public:
  gps_to_odom() {
    gps_sub = n.subscribe("/fix", 1, &gps_to_odom::gps_callback, this);
  }

  void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    ROS_INFO("Recieved msg :D");

    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    // convert from latitude-longitude-altitude to Cartesian ECEF
    double R = a / sqrt(1 - pow(e, 2) * pow(sin(lat), 2));
    double X = (R + alt) * cos(lat) * cos(lon);
    double Y = (R + alt) * cos(lat) * sin(lon);
    double Z = ((pow(a, 2) / pow(b, 2)) * R + alt) * sin(lat);
  
    double X_zero = (R + alt_zero) * cos(lat_zero) * cos(lon_zero);
    double Y_zero = (R + alt_zero) * cos(lat_zero) * sin(lon_zero);
    double Z_zero = ((pow(a, 2) / pow(b, 2)) * R + alt_zero) * sin(lat_zero);

    double X_diff = X - X_zero;
    double Y_diff = Y - Y_zero;
    double Z_diff = Z - Z_zero;
    
    // convert from Cartesian ECEF to NED
    double N = (- sin(lat_zero) * cos(lon_zero)) * X_diff + (- sin(lat_zero) * sin(lon_zero)) * Y_diff + (cos(lat_zero)) * Z_diff;
    double E = (- sin(lon_zero)) * X_diff + (cos(lon_zero)) * Y_diff + (0) * Z_diff;
    double D = (- cos(lat_zero) * cos(lon_zero)) * X_diff + (- cos(lat_zero) * sin(lon_zero)) * Y_diff + (- sin(lat_zero)) * Z_diff;

    // construct the odom message
    nav_msgs::Odometry odom_msg;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quat_msg;
    tf2::Quaternion quat_tf;

    point.x = N;
    point.y = E;
    point.z = D;
    odom_msg.pose.pose.position = point;

    // estimate heading from consecutive poses
    double heading = atan2(N - N_prev, - (E - E_prev));
    quat_tf.setRPY(0, 0, heading);
    quat_msg = tf2::toMsg(quat_tf);
    odom_msg.pose.pose.orientation = quat_msg;

    // publish
    gps_odom_pub.publish(odom_msg);

    // update
    N_prev = N;
    E_prev = E;
    D_prev = D;
  }

};


int main(int argc, char **argv){

    ros::init(argc, argv, "gps_to_odom");
    gps_to_odom gps_to_odom;
    //ros::Rate loop_rate(10);

    ROS_INFO("%s", "hello gps_to_odom!");

  	while (ros::ok()){
    	
    	ros::spinOnce();
      //loop_rate.sleep();
    }
  	return 0;
}