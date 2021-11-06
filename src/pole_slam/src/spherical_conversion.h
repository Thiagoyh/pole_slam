#pragma once

#include <math.h>
#include <opencv2/opencv.hpp> // For visualizing image
#include <pcl/point_cloud.h>  // For PCL Point Cloud
#include <pcl/io/pcd_io.h>    // For Reading the Cloud
#include <pcl/point_types.h>  // For PCL different cloud types
#include <algorithm>
#include <vector>

#include "point_type/common.h"
#include "options.h"

namespace spherical{
struct Configuration
{
    Configuration() = default;
    Configuration(const ::config_lua::NodeOptions& options):
    fov_up(options.fov_up), fov_down(options.fov_down),
    img_height(options.img_height), img_width(options.img_width){}

    double fov_up;
    double fov_down;
    double img_height;
    double img_width;
};

class SphericalConversion
{
public:
    explicit SphericalConversion(const Configuration &config);
    SphericalConversion(SphericalConversion&) = delete;
    SphericalConversion &operator=(SphericalConversion &) = delete;

    int LoadCloud(const std::string &path);
    void MakeImage(bool has_intensity);
    void GetProjection(const PointType &point,
                       const double &fov_rad,
                       const double &fov_down_rad,
                       int *pixel_v, int *pixel_u,
                       double *range) const;
    std::vector<std::vector<std::vector<double>>> GetFullImg() const;
    std::vector<std::vector<double>> GetDepthImg() const;

    void ShowImg(const std::vector<std::vector<double>>& depth_img_) const;
    void GetCloud(const pcl::PointCloud<PointType>::Ptr &cloud);

private:
    Configuration config_;
    pcl::PointCloud<PointType>::Ptr cloud_;
    std::vector<std::vector<std::vector<double>>> spherical_img_;
    std::vector<std::vector<double>> depth_img_;
};
}