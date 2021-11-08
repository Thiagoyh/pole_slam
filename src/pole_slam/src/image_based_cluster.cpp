#include "image_based_cluster.h"
#include <cmath>

namespace image_cluster
{
      // defination of conduct function
      ImageCluster::ImageCluster(const config_lua::NodeOptions& options):
           height(options.img_height),
           width(options.img_width),
           min_point_num(options.min_cluster_num),
           dis_thr(options.min_cluster_dis),
           width_thr(options.width_thr)
           {
               std::cout << options.min_cluster_dis << std::endl;
               std::cout << options.min_cluster_num << std::endl;
           }

      // to find index in the set
      bool ImageCluster::in_set(const std::set<Index>& set, Index index)
     {
         std::set<Index>::iterator iter = set.find(index);
         if(iter != set.end())
             return true;
         return false;
     }

      std::set<Index> ImageCluster::Get_open_set(const DepthType& range_data)
      {
        std::set<Index> open_set;
        for (size_t i = 1; i != height - 1; ++i)
        {
            for (DepthType::size_type j = 1; j != width - 1; ++j)
            {
                if(range_data[i][j] != 0)
                    open_set.insert(Index{i, j});
            }
        }
        return open_set;
      }

    std::vector<Cluster> ImageCluster::gen_clusters(const DepthType& range_data)
    {
        std::vector<Cluster> clusters;
        std::set<Index> open_set = Get_open_set(range_data);

        while(!open_set.empty())
        {
            std::set<Index> cluster;
            Index current_index = *(open_set.begin());

            open_set.erase(open_set.begin());
            cluster.insert(current_index);

            std::set<Index> near_set;

            if(current_index.first + 1 < height &&
            in_set(open_set, Index{current_index.first + 1, current_index.second}) &&
            std::fabs(range_data[current_index.first][current_index.second] -
            range_data[current_index.first + 1][current_index.second]) < dis_thr)
            {
                near_set.insert(Index{current_index.first + 1, current_index.second});
            }

            if(current_index.second + 1 < width &&
            in_set(open_set, Index{current_index.first, current_index.second + 1}) &&
            std::fabs(range_data[current_index.first][current_index.second]) -
            range_data[current_index.first][current_index.second + 1] < dis_thr)
            {
                near_set.insert(Index{current_index.first, current_index.second + 1});
            }

            while(!near_set.empty())
            {
                Index near_index = *(near_set.begin());
                near_set.erase(near_set.begin());

                std::set<Index>::iterator iter = open_set.find(near_index);
                if(iter != open_set.end())
                    open_set.erase(iter);
                cluster.insert(near_index);

                if (near_index.first + 1 < height &&
                    in_set(open_set, Index{near_index.first + 1, near_index.second}) &&
                    !in_set(cluster, Index{near_index.first + 1, near_index.second}) &&
                    !in_set(near_set, Index{near_index.first + 1, near_index.second}) &&
                    std::fabs(range_data[near_index.first][near_index.second] -
                    range_data[near_index.first + 1][near_index.second]) < dis_thr)
                    {
                        near_set.insert(Index{near_index.first + 1, near_index.second});
                    }

                if(near_index.second + 1 < width &&
                in_set(open_set, Index{near_index.first, near_index.second + 1}) &&
                !in_set(cluster, Index{near_index.first, near_index.second + 1}) &&
                !in_set(near_set, Index{near_index.first, near_index.second + 1}) &&
                std::fabs(range_data[near_index.first][near_index.second] -
                    range_data[near_index.first][near_index.second + 1]) < dis_thr)
                    {
                        near_set.insert(Index{near_index.first, near_index.second + 1});
                    }

                if(near_index.second - 1 >= 0 &&
                in_set(open_set, Index{near_index.first, near_index.second - 1}) &&
                !in_set(cluster, Index{near_index.first, near_index.second - 1}) &&
                !in_set(near_set, Index{near_index.first, near_index.second - 1}) &&
                std::fabs(range_data[near_index.first][near_index.second] -
                    range_data[near_index.first][near_index.second - 1]) < dis_thr)
                    {
                        near_set.insert(Index{near_index.first, near_index.second - 1});
                    }
            }
            if(cluster.size() > min_point_num)
                clusters.push_back(cluster);
        }
        return clusters;
    }

    std::vector<Cluster> ImageCluster::remove_noise(const DepthType& range_data, const std::vector<Cluster>& clusters)
    {
        //std::vector <std::vector<Index>> clusters_vec;

        std::vector<Cluster> clusters_filtered;
        clusters_filtered.reserve(clusters.size());

        for (std::vector<Cluster>::const_iterator iter = clusters.begin(); iter != clusters.end(); ++iter)
        {
            int min_height = iter->begin()->first;
            int max_height = (--iter->end())->first;

            int min_width = 1024;
            int max_width = 1;

            for (Cluster::iterator it_cluster = iter->begin(); it_cluster != iter->end(); ++it_cluster)
            {
                //cluster_vec.push_back(*it_cluster);
                if(it_cluster->second < min_width)
                    min_width = it_cluster->second;
                if(it_cluster->second > max_width)
                    max_width = it_cluster->second;
            }


            double ratio = (max_height - min_height + 1) / (max_width - min_width + 1);

            int delate = 0;
            bool dela = false;

            for (Cluster::iterator it_cluster = iter->begin(); it_cluster != iter->end(); ++it_cluster)
            {
                if(range_data[it_cluster->first][it_cluster->second + 1] != 0 &&
                 !in_set(*iter, Index{it_cluster->first, it_cluster->second + 1}) &&
                 range_data[it_cluster->first][it_cluster->second] > range_data[it_cluster->first][it_cluster->second + 1])
                {
                    dela = true;
                }

                if(range_data[it_cluster->first][it_cluster->second - 1] != 0 &&
                !in_set(*iter, Index{it_cluster->first, it_cluster->second - 1}) &&
                 range_data[it_cluster->first][it_cluster->second] > range_data[it_cluster->first][it_cluster->second - 1])
                {
                    dela = true;
                }

                if(dela)
                {
                    delate += 1;
                    dela = false;
                }

            }
            if (!(ratio < 1.0 || (delate > 0.3 * iter->size()) || (max_width - min_width + 1) > width_thr))
                clusters_filtered.push_back(*iter);
        }
        return clusters_filtered;
    }
}