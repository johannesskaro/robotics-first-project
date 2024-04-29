#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class lidar_remap {

  private:
    ros::NodeHandle n;
    ros::Subscriber os_cloud_sub;
    ros::Publisher lidar_remap_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1);
    std::string frame_id;

  public:
    lidar_remap() {
      os_cloud_sub = n.subscribe("/os_cloud_node/points", 1, &lidar_remap::os_cloud_callback, this);
    }

    void os_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
      sensor_msgs::PointCloud2 point_cloud = *msg;
      point_cloud.header.frame_id = frame_id;
      point_cloud.header.stamp = ros::Time::now();
      lidar_remap_pub.publish(point_cloud);
    }
    void param_callback(first_project::parametersConfig &config, uint32_t level) {
      ROS_INFO("Reconfigure Request: %s", config.frame_id_param.c_str());
      frame_id = config.frame_id_param;
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_remap");
    lidar_remap lidar_remap;
    ROS_INFO("%s", "Running lidar_remap");

    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
    f = boost::bind(&lidar_remap::param_callback, &lidar_remap, _1, _2);
    server.setCallback(f);

  	while (ros::ok()){
    	ros::spinOnce();
    }
  	return 0;
}