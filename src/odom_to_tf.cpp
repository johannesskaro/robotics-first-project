#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class odom_to_tf{
public:
    odom_to_tf(){
      sub = n.subscribe("/input_odom", 1, &odom_to_tf::callback, this);
      std::string root_frame_param_name = ros::this_node::getName() + "/root_frame";
      std::string child_frame_param_name = ros::this_node::getName() + "/child_frame"; 
      n.getParam(root_frame_param_name, root_frame);
      n.getParam(child_frame_param_name, child_frame);
      ROS_INFO("%s", root_frame.c_str());
      ROS_INFO("%s", child_frame.c_str());
    }
    
void callback(const nav_msgs::Odometry::ConstPtr& msg){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg -> pose.pose.position.x, msg -> pose.pose.position.y, 0));
  tf::Quaternion q( msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
  };

private:
  ros::NodeHandle n;
  tf::TransformBroadcaster br;
  ros::Subscriber sub;
  std::string root_frame;
  std::string child_frame;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "odom_to_tf");
    odom_to_tf odom_to_tf;
    ROS_INFO("%s", "hello odom_to_tf!");

  	while (ros::ok()){
    	ros::spinOnce();
    }
  	return 0;
}