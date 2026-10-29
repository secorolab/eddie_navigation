#include "simulation/eddie_simulation_plugin.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eddie_driver/io_mujoco.h>
#include <pluginlib/class_list_macros.hpp>
#include <map>
#include <stdexcept>

namespace eddie_navigation {

void EddieSimulationPlugin::configure(mj_kdl::SceneSpec& scene, rclcpp::Node& host)
{
    if (!scene.robots.empty()) throw std::runtime_error("Eddie plugin expects no prior robot");
    const double kv = host.declare_parameter("sim_wheel_kv", 16.0);
    const auto model = ament_index_cpp::get_package_share_directory("eddie_description")
                     + "/mujoco/mobile_platform.xml";
    const auto eddie = eddie_io_mujoco_scene(model.c_str(), kv);
    scene.robots = eddie.robots;
}

void EddieSimulationPlugin::initialize(mj_kdl::Env& env, rclcpp::Node& host)
{
    env_ = &env;
    std::map<std::string, rclcpp::Parameter> values;
    for (const auto& value : host.get_node_parameters_interface()->get_parameter_overrides())
        values.emplace(value.first, rclcpp::Parameter(value.first, value.second));
    for (const auto& name : host.list_parameters({}, 0).names)
        values.insert_or_assign(name, host.get_parameter(name));
    values.insert_or_assign("io", rclcpp::Parameter("io", "mujoco"));
    std::vector<rclcpp::Parameter> parameters;
    for (const auto& value : values) parameters.push_back(value.second);

    // Separate ROS node, same process and same Env. Preserve the existing /cmd_vel interface.
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides(parameters)
           .arguments({"--ros-args", "-r", "/platform_vel:=/cmd_vel"});
    adapter_ = eddie_driver_ros::make_simulation_adapter(env, options);
}

void EddieSimulationPlugin::afterStep()
{
    if (env_->data->time - last_update_ + 1e-9 < adapter_.period) return;
    last_update_ = env_->data->time;
    adapter_.update();
}

void EddieSimulationPlugin::reset(mj_kdl::ResetContext* context)
{
    last_update_ = context->data->time;
    adapter_.reset(context);
}

std::vector<rclcpp::Node::SharedPtr> EddieSimulationPlugin::nodes() const
{
    return {adapter_.node};
}

}  // namespace eddie_navigation

PLUGINLIB_EXPORT_CLASS(eddie_navigation::EddieSimulationPlugin, bim_experiments::SimulationPlugin)
