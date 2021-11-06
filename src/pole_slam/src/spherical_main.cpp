#include <iostream>
#include "spherical_conversion.h"
#include "subscriber.h"
#include "options.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

/** For Velodyne HDL 64-E
   * @param: Fov_Up = 2 degrees
   * @param:Fov_Down = -24.8 degrees
   * @param:Num of Lasers = 64
   * @param:Best length of image comes out to be = 1024
*/

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::GLOG_INFO);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::config_lua::NodeOptions node_options =
          ::config_lua::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  //const spherical::Configuration config_input(node_options);

  ros::init(argc, argv, "spherical");
  pole_slam::LidarProcess lidarprocess_(node_options);
  ros::spin();
  return 0;
}