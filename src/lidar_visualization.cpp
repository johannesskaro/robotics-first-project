#include "ros/ros.h"

// write third node here

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_visualization");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

  	while (ros::ok()){

    	ROS_INFO("%s", "hello lidar_visualization!");

    	ros::spinOnce();
        loop_rate.sleep();
    }
  	return 0;
}