#include "spherical_conversion.h"


namespace spherical{
SphericalConversion::SphericalConversion(const Configuration& config)
    : config_(config) {
  spherical_img_.assign(config_.img_height,
                        std::vector<std::vector<double>>(
                            config_.img_width, std::vector<double>(5, 0.0)));

  depth_img_.assign(config_.img_height, std::vector<double>(config_.img_width, 0.0));

  cloud_ =
      pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
};


int SphericalConversion::LoadCloud(const std::string& path) {
  // Loading from Bin File
  if (pcl::io::loadPCDFile<PointType>(path, *cloud_) == -1) {
    std::cout << "Couldn't read the cloud file at: " << path << "\n";
    return -1;
  }
  return 1;
}


void SphericalConversion::GetCloud(const pcl::PointCloud<PointType>::Ptr& cloud)
{
    cloud_ = cloud;
}

void SphericalConversion::MakeImage(bool has_intensity) {
  // Converting to Radians
  double fov_up_rad = (config_.fov_up / 180) * M_PI;
  double fov_down_rad = (config_.fov_down / 180) * M_PI;
  // Getting total Field of View
  double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
  if (cloud_->size() == 0) {
    std::cerr << "Empty Point Cloud_" << std::endl;
    return;
  }
  for (auto point : *cloud_) {
    // Getting Pixel from Point
    int pixel_v = 0;
    int pixel_u = 0;
    double range = 0.0;
    GetProjection(point, fov_rad, fov_down_rad, &pixel_v, &pixel_u, &range);

    spherical_img_.at(pixel_u).at(pixel_v) =
        std::vector<double>{point.x, point.y, point.z, range};

    depth_img_.at(pixel_u).at(pixel_v) = range;
  }
}

void SphericalConversion::GetProjection(const PointType& point,
                                        const double& fov_rad,
                                        const double& fov_down_rad,
                                        int* pixel_v, int* pixel_u,
                                        double* range) const {
  // range of Point from Lidar
  *range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  //  Getting the angle of all the Points
  auto yaw = atan2(point.y, point.x);
  auto pitch = asin(point.z / *range);
  // Get projections in image coords and normalizing
  double v = 0.5 * (yaw / M_PI + 1.0);
  double u = 1.0 - (pitch + std::abs(fov_down_rad)) / fov_rad;
  // Scaling as per the lidar config given
  v *= config_.img_width;
  u *= config_.img_height;
  // round and clamp for use as index
  v = floor(v);
  v = std::min(config_.img_width - 1, v);
  v = std::max(0.0, v);
  *pixel_v = int(v);

  u = floor(u);
  u = std::min(config_.img_height - 1, u);
  u = std::max(0.0, u);
  *pixel_u = int(u);
}

std::vector<std::vector<std::vector<double>>> SphericalConversion::GetFullImg() const
{
   return spherical_img_;
}

std::vector<std::vector<double>> SphericalConversion::GetDepthImg() const
{
  return depth_img_;
}

void SphericalConversion::ShowImg(
    const std::vector<std::vector<double>>& depth_img_) const {
  cv::Mat sp_img(depth_img_.size(), depth_img_.at(0).size(), CV_64FC1);
  for (int i = 0; i < sp_img.rows; ++i) {
    for (int j = 0; j < sp_img.cols; ++j) {
      sp_img.at<double>(i, j) = depth_img_[i][j]; //Intensity value
    }
  }
  double min, max;
  cv::minMaxIdx(sp_img, &min, &max);
  cv::Mat adjImg;
  cv::convertScaleAbs(sp_img, adjImg, 255 / max);

  cv::Mat fasleColorsMap;
  cv::applyColorMap(adjImg, fasleColorsMap, cv::COLORMAP_JET);

  cv::imshow("color", fasleColorsMap);

  cv::imshow("depth Image", adjImg);
  cv::waitKey(0);
}
}