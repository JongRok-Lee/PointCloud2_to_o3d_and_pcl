#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_dst;
    pcl::fromROSMsg(cloudmsg, cloud_dst);
    return cloud_dst;
  }

void callback(sensor_msgs::PointCloud2 msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud =  cloudmsg2cloud(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, callback);
    ros::spin();
    
    return 0;   
}
