#pragma once

#include <pcl/point_types.h>

typedef pcl::PointXYZ PointType;

typedef std::vector<std::vector<double>> DepthType;
typedef std::vector<std::vector<std::vector<double>>> DepthFullType;

typedef std::pair<int, int> Index;
typedef std::set<std::pair<int, int>> Cluster;
