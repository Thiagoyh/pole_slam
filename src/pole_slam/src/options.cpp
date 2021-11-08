#include "options.h"

#include <vector>
#include <memory>
#include "absl/memory/memory.h"
#include "common/configuration_file_resolver.h"
#include "glog/logging.h"

namespace config_lua {

/**
 * @brief 读取lua文件内容, 将lua文件的内容赋值给NodeOptions
 *
 * @param lua_parameter_dictionary lua字典
 * @return NodeOptions
 */
NodeOptions CreateNodeOptions(
    common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  NodeOptions options;
  // 根据lua字典中的参数, 生成protobuf的序列化数据结构 proto::MapBuilderOptions

  options.fov_up =
      lua_parameter_dictionary->GetDouble("fov_up");
  options.fov_down =
      lua_parameter_dictionary->GetDouble("fov_down");
  options.img_height =
      lua_parameter_dictionary->GetInt("num_lasers");
  options.img_width =
      lua_parameter_dictionary->GetInt("img_width");

  options.min_cluster_dis =
      lua_parameter_dictionary->GetDouble("min_cluster_dis");
  options.min_cluster_num =
      lua_parameter_dictionary->GetInt("min_cluster_num");

  options.width_thr =
      lua_parameter_dictionary->GetInt("width_threshold");
//   if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
//     options.publish_to_tf = lua_parameter_dictionary->GetBool("publish_to_tf");
//   }
//   if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
//     options.publish_tracked_pose =
//         lua_parameter_dictionary->GetBool("publish_tracked_pose");
//   }
//   if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
//     options.use_pose_extrapolator =
//         lua_parameter_dictionary->GetBool("use_pose_extrapolator");
// }
  return options;
}

/**
 * @brief 加载lua配置文件中的参数
 *
 * @param[in] configuration_directory 配置文件所在目录
 * @param[in] configuration_basename 配置文件的名字
 * @return std::tuple<NodeOptions, TrajectoryOptions> 返回节点的配置与轨迹的配置
 */


NodeOptions LoadOptions(const std::string &configuration_directory,
                        const std::string &configuration_basename)
{
      auto file_resolver =
      absl::make_unique<common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});

    const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);

      common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

      return CreateNodeOptions(&lua_parameter_dictionary);
}

}  // namespace cartographer_ros
