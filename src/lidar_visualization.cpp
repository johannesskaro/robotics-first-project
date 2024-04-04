#include "ros/ros.h"

// write third node here

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_visualization");

  	while (ros::ok()){

    	ROS_INFO("%s", "hello lidar_visualization!");


    	ros::spinOnce();
  	
    }
  	return 0;
}