#pragma once

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include "spherical_conversion.h"
#include <memory>
#include <iostream>

#include "image_based_cluster.h"

namespace pole_slam{

class LidarProcess
{
public:
    // lidar_config conduct function
    LidarProcess(const spherical::Configuration &config);
    // lua_options conduct function
    LidarProcess(const config_lua::NodeOptions& options);

    // deconduct function is default
    ~LidarProcess() = default;
    // callback
    void HandleLidarMessage(const sensor_msgs::PointCloud2ConstPtr &msg);
    ros::NodeHandle nh_;

private:
    spherical::SphericalConversion spherical_;
    image_cluster::ImageCluster cluster_;

    ros::Subscriber lidar_sub;
};

}