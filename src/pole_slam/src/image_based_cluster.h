
#pragma once

#include <vector>
#include <set>

#include "point_type/common.h"
#include "spherical_conversion.h"
#include "options.h"

/**
 * @param min_point_num: the minimum number of image cluster
 * @param dis_thr: the threshold of distance when clustering
 * @param width_thr: width threshole to remove the clusters which hardly looks like pole-feature;
 * @param height: the height of depth image
 * @param width: the width of depth image
 */
namespace image_cluster
{
    class ImageCluster
    {
    public:
        ImageCluster() = default;
        ImageCluster(const config_lua::NodeOptions& options);

        ImageCluster(ImageCluster &) = delete;
        ImageCluster &operator=(ImageCluster) = delete;
        ~ImageCluster() = default;

        bool in_set(const std::set<Index>& set, Index index);
        std::set<Index> Get_open_set(const DepthType& range_data);
        std::vector<Cluster> gen_clusters(const DepthType& range_data);

        std::vector<Cluster> remove_noise(const DepthType& range_data, const std::vector<Cluster>& clusters);


    private:

        int min_point_num = 3;
        double dis_thr = 0.08;
        int height = 64;
        int width = 1024;

        int width_thr = 10;
    };
}
