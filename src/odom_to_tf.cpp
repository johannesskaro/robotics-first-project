#include "ros/ros.h"

// write second node here

int main(int argc, char **argv){

    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

  	while (ros::ok()){

    	ROS_INFO("%s", "hello odom_to_tf!");

    	ros::spinOnce();
        loop_rate.sleep();
    }
  	return 0;
}