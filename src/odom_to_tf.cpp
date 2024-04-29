#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

class tf_sub_pub{
public:
    tf_sub_pub(){
      sub = n.subscribe("input_odom", 1000, &tf_sub_pub::callback, this);
    }


void callback(const nav_msgs::Odometry::ConstPtr& msg){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg -> pose.pose.position.x, msg -> pose.pose.position.y, 0));
  tf::Quaternion q(msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root_frame","child_frame"));
   };

private:
  ros::NodeHandle n;
  tf::TransformBroadcaster br;
  ros::Subscriber sub;
};

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