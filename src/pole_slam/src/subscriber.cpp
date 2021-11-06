#include "subscriber.h"
#include <memory>

namespace{
  bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}
}
namespace pole_slam{

LidarProcess::LidarProcess(const spherical::Configuration &config) :
                                                         spherical_(config)
{
    lidar_sub = nh_.subscribe("/kitti/velo/pointcloud", 1, &LidarProcess::HandleLidarMessage, this);
}

LidarProcess::LidarProcess(const config_lua::NodeOptions &options) : spherical_(spherical::Configuration(options))
{
    lidar_sub = nh_.subscribe("/kitti/velo/pointcloud", 1, &LidarProcess::HandleLidarMessage, this);
}

void LidarProcess::HandleLidarMessage(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<PointType>::Ptr lidar_message(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *lidar_message);
    spherical_.GetCloud(lidar_message);
    spherical_.MakeImage(PointCloud2HasField(*msg, "intensity"));
    auto img = spherical_.GetDepthImg();
    spherical_.ShowImg(img);


}

}