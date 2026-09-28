#include <eddie_navigation/action/constrained_pass.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>
#include <vector>

namespace eddie_navigation
{

using ConstrainedPassGoal = action::ConstrainedPass::Goal;

// The first zone the path crosses, as a ConstrainedPass goal and the pose before it; else FAILURE.
class ZoneOnPath : public BT::SyncActionNode
{
public:
    ZoneOnPath(const std::string &name, const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(name, conf)
    {
        auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node->has_parameter("zones_file")) {
            node->declare_parameter("zones_file", std::string(""));
        }
        const std::string file = node->get_parameter("zones_file").as_string();
        if (!file.empty()) load(file);
        RCLCPP_INFO(node->get_logger(), "ZoneOnPath: %zu zones from '%s'", zones_.size(),
                file.c_str());
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<nav_msgs::msg::Path>("path"),
            BT::InputPort<double>("approach", 0.6, "[m] zone centre to where the pass starts"),
            BT::OutputPort<ConstrainedPassGoal>("zone"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_pose"),
        };
    }

    BT::NodeStatus tick() override
    {
        nav_msgs::msg::Path path;
        double approach = 0.6;
        if (!getInput("path", path) || path.poses.size() < 2) return BT::NodeStatus::FAILURE;
        getInput("approach", approach);
        for (size_t i = 0; i + 1 < path.poses.size(); i++) {
            const auto &a = path.poses[i].pose.position, &b = path.poses[i + 1].pose.position;
            for (const auto &z : zones_) {
                const double sa = (a.x - z.goal.centre.x) * z.nx + (a.y - z.goal.centre.y) * z.ny;
                const double sb = (b.x - z.goal.centre.x) * z.nx + (b.y - z.goal.centre.y) * z.ny;
                if (sa * sb > 0 || sa == sb) continue;
                const double t = sa / (sa - sb);
                const double x = a.x + t * (b.x - a.x), y = a.y + t * (b.y - a.y);
                const double off = (x - z.goal.centre.x) * -z.ny + (y - z.goal.centre.y) * z.nx;
                if (std::fabs(off) >= z.goal.width / 2) continue;

                const double s = sb > sa ? 1.0 : -1.0;
                ConstrainedPassGoal goal = z.goal;
                goal.direction.x = s * z.nx;
                goal.direction.y = s * z.ny;
                goal.approach = approach;
                geometry_msgs::msg::PoseStamped before;
                before.header = path.header;
                before.pose.position.x = z.goal.centre.x - goal.direction.x * approach;
                before.pose.position.y = z.goal.centre.y - goal.direction.y * approach;
                const double yaw = std::atan2(goal.direction.y, goal.direction.x);
                before.pose.orientation.z = std::sin(yaw / 2);
                before.pose.orientation.w = std::cos(yaw / 2);
                setOutput("zone", goal);
                setOutput("approach_pose", before);
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    struct Zone
    {
        ConstrainedPassGoal goal;
        double nx, ny;
    };

    void load(const std::string &file)
    {
        for (const auto &z : YAML::LoadFile(file)["zones"]) {
            Zone zone;
            zone.goal.name = z["name"].as<std::string>();
            zone.goal.centre.x = z["centre"][0].as<double>();
            zone.goal.centre.y = z["centre"][1].as<double>();
            zone.goal.width = z["width"].as<double>();
            zone.nx = z["normal"][0].as<double>();
            zone.ny = z["normal"][1].as<double>();
            const auto c = z["constraints"];
            auto &mc = zone.goal.constraints;
            mc.heading_mode = c["heading_mode"].as<std::string>() == "forward"
                    ? msg::MotionConstraints::HEADING_FORWARD
                    : msg::MotionConstraints::HEADING_ANY;
            mc.max_speed = c["max_speed"].as<double>();
            mc.align_tolerance_xy = c["align_tolerance_xy"].as<double>();
            mc.align_tolerance_yaw = c["align_tolerance_yaw"].as<double>();
            mc.stop_distance = c["stop_distance"].as<double>();
            zones_.push_back(zone);
        }
    }

    std::vector<Zone> zones_;
};

class ConstrainedPassAction : public nav2_behavior_tree::BtActionNode<action::ConstrainedPass>
{
public:
    ConstrainedPassAction(const std::string &xml_tag_name, const std::string &action_name,
            const BT::NodeConfiguration &conf)
        : BtActionNode<action::ConstrainedPass>(xml_tag_name, action_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({ BT::InputPort<ConstrainedPassGoal>("zone") });
    }

    void on_tick() override { getInput("zone", goal_); }
};

}  // namespace eddie_navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<eddie_navigation::ZoneOnPath>("ZoneOnPath");
    factory.registerBuilder<eddie_navigation::ConstrainedPassAction>("ConstrainedPass",
            [](const std::string &name, const BT::NodeConfiguration &config) {
                return std::make_unique<eddie_navigation::ConstrainedPassAction>(
                        name, "constrained_pass", config);
            });
}
