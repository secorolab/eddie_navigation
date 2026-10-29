#pragma once

#include <simulation/simulation_plugin.hpp>
#include <eddie_driver_ros/simulation_adapter.hpp>

namespace eddie_navigation {

class EddieSimulationPlugin : public bim_experiments::SimulationPlugin {
public:
    void configure(mj_kdl::SceneSpec& scene, rclcpp::Node& host) override;
    void initialize(mj_kdl::Env& env, rclcpp::Node& host) override;
    void afterStep() override;
    void reset(mj_kdl::ResetContext* context) override;
    std::vector<rclcpp::Node::SharedPtr> nodes() const override;

private:
    mj_kdl::Env* env_ = nullptr;
    eddie_driver_ros::SimulationAdapter adapter_{};
    double last_update_ = 0.0;
};

}  // namespace eddie_navigation
