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

  double lat_ref_deg;
  double lon_ref_deg;
  double alt_ref_m;

  double a = 6378137; // Semi-major axis of earth given in meters
  double b = 6356752; // Semi-minor axis of earth given in meters
  double e_squared = 1 - pow(b, 2) / pow(a, 2);  //  0.0818191908426; // Eccentricity of earth

  double N_prev = 0;
  double E_prev = 0;
  double U_prev = 0;

public:
  gps_to_odom() {
    gps_sub = n.subscribe("/fix", 1, &gps_to_odom::gps_callback, this);
    n.getParam("/gps_to_odom/lat_r", lat_ref_deg);
    n.getParam("/gps_to_odom/lon_r", lon_ref_deg);
    n.getParam("/gps_to_odom/alt_r", alt_ref_m);
  }

  void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    double lat = msg->latitude * (M_PI / 180);
    double lon = msg->longitude * (M_PI / 180);
    double alt = msg->altitude;

    double lat_ref = lat_ref_deg * (M_PI / 180);
    double lon_ref = lon_ref_deg * (M_PI / 180);
    double alt_ref = alt_ref_m;

    // convert from latitude-longitude-altitude to Cartesian ECEF
    double R = a / sqrt(1 - e_squared * pow(sin(lat), 2));
    double X = (R + alt) * cos(lat) * cos(lon);
    double Y = (R + alt) * cos(lat) * sin(lon);
    double Z = (R * (1 - e_squared) + alt) * sin(lat);

    double R_ref = a / sqrt(1 - e_squared * pow(sin(lat_ref), 2));
    double X_ref = (R_ref + alt_ref) * cos(lat_ref) * cos(lon_ref);
    double Y_ref = (R_ref + alt_ref) * cos(lat_ref) * sin(lon_ref);
    double Z_ref = (R_ref * (1 - e_squared) + alt_ref) * sin(lat_ref);

    double X_diff = X - X_ref;
    double Y_diff = Y - Y_ref;
    double Z_diff = Z - Z_ref;
    
    // convert from Cartesian ECEF to ENU
    double E = (- sin(lon_ref)) * X_diff + (cos(lon_ref)) * Y_diff + (0) * Z_diff;
    double N = (- sin(lat_ref) * cos(lon_ref)) * X_diff + (- sin(lat_ref) * sin(lon_ref)) * Y_diff + (cos(lat_ref)) * Z_diff;
    double U = (cos(lat_ref) * cos(lon_ref)) * X_diff + (cos(lat_ref) * sin(lon_ref)) * Y_diff + (sin(lat_ref)) * Z_diff;

    // construct the odom message
    nav_msgs::Odometry odom_msg;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quat_msg;
    tf2::Quaternion quat_tf;

    point.x = E;
    point.y = N;
    point.z = U;
    odom_msg.pose.pose.position = point;

    // estimate heading from consecutive poses
    double heading = atan2(N - N_prev, E - E_prev);
    //ROS_INFO("%f", heading);
    quat_tf.setRPY(0, 0, heading);
    quat_msg = tf2::toMsg(quat_tf);
    odom_msg.pose.pose.orientation = quat_msg;

    // publish
    // odom_msg.child_frame_id = "reach_rs";
    // odom_msg.header.frame_id = "reach_rs";
    gps_odom_pub.publish(odom_msg);

    // update
    E_prev = E;
    N_prev = N;
    U_prev = U;
  }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "gps_to_odom");
    gps_to_odom gps_to_odom;
    ROS_INFO("%s", "Running gps_to_odom");

  	while (ros::ok()){
    	ros::spinOnce();
    }
  	return 0;
}