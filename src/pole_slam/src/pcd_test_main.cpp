#include "spherical_conversion.h"
#include "ros/ros.h"

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "options.h"

#include "image_based_cluster.h"

DEFINE_string(path, "",
            "the pcd location!");

DEFINE_string(configuration_directory, "1",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "2",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

int main(int argc, char**argv)
{
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_INFO);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "spherical_");
    ::config_lua::NodeOptions node_options = ::config_lua::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    const spherical::Configuration config_(node_options);
    const std::string path = "/home/xcy/bag/test_cloud.pcd";

    std::cout << "path: " << FLAGS_path << std::endl;
    std::cout << "directory: " << FLAGS_configuration_directory << std::endl;
    std::cout << "basename: " << FLAGS_configuration_basename << std::endl;


    spherical::SphericalConversion conv(config_);
    image_cluster::ImageCluster cluster_(node_options);

    conv.LoadCloud(path);
    conv.MakeImage(true);
    auto img = conv.GetDepthImg();
    conv.ShowImg(img);

    std::vector<Cluster> clusters = cluster_.gen_clusters(img);


    LOG(INFO) << clusters.size();


    std::vector<Cluster>::iterator test_it = clusters.begin();

    for (Cluster::iterator it_cluster_ = test_it->begin(); it_cluster_ != test_it->end(); ++it_cluster_)
    {
        std::cout << it_cluster_->first << " " << it_cluster_->second << std::endl;
    }


    for (std::vector<Cluster>::iterator iter = clusters.begin(); iter != clusters.end(); ++iter)
    {
        cv::Mat sp_img(img.size(), img.at(0).size(), CV_64FC1);
        for (Cluster::iterator it_cluster = iter->begin(); it_cluster != iter->end(); ++it_cluster)
        {
            sp_img.at<double>(it_cluster->first, it_cluster->second) = img[it_cluster->first][it_cluster->second];
        }

        double min, max;
        cv::minMaxIdx(sp_img, &min, &max);

        std::cout << "max1: " << max << std::endl;
        std::cout << "min1: " << min << std::endl;
        cv::Mat adjimg;
        cv::convertScaleAbs(sp_img, adjimg, 255 / max);

        std::string img_name = "cluster" + std::to_string(iter - clusters.begin());

        //  cv::imshow(img_name, adjimg);
        //  cv::waitKey(0);

        if(iter == (--clusters.end()))
        {
            cv::imshow(img_name, adjimg);
            cv::waitKey(0);
        }
    }

    std::vector<Cluster> clusters_filtered = cluster_.remove_noise(img, clusters);
    LOG(INFO) << clusters_filtered.size();

    double rawData[1024][64] = {};
    cv::Mat sp_img_(img.size(), img.at(0).size(), CV_64FC1, rawData);
    for (std::vector<Cluster>::iterator iter = clusters_filtered.begin(); iter != clusters_filtered.end(); ++iter)
    {
        for (Cluster::iterator it_cluster = iter->begin(); it_cluster != iter->end(); ++it_cluster)
        {
            sp_img_.at<double>(it_cluster->first, it_cluster->second) = img[it_cluster->first][it_cluster->second];
        }

        double min, max;
        cv::minMaxIdx(sp_img_, &min, &max);
        std::cout << "max: " << max << std::endl;
        std::cout << "min: " << min << std::endl;
        cv::Mat adjimg;
        cv::convertScaleAbs(sp_img_, adjimg, 255 / max);

        std::string img_name = "cluster_1" + std::to_string(iter - clusters_filtered.begin());

        //  cv::imshow(img_name, adjimg);
        //  cv::waitKey(0);

        if(iter == --clusters_filtered.end())
        {
            cv::imshow(img_name, adjimg);
            cv::waitKey(0);
        }
    }

        return 0;
}