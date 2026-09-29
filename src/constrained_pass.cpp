#include <eddie_navigation/action/constrained_pass.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace eddie_navigation
{
namespace
{

constexpr int8_t LETHAL = 100;  // OccupancyGrid value of a lethal cell

struct Vec
{
    double x = 0, y = 0;
    Vec operator+(Vec o) const { return { x + o.x, y + o.y }; }
    Vec operator-(Vec o) const { return { x - o.x, y - o.y }; }
    Vec operator*(double k) const { return { x * k, y * k }; }
    double dot(Vec o) const { return x * o.x + y * o.y; }
    double norm() const { return std::hypot(x, y); }
    Vec perp() const { return { -y, x }; }
};

double clip(double v, double limit) { return std::clamp(v, -limit, limit); }

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

bool lethal(const nav_msgs::msg::OccupancyGrid &m, Vec p)
{
    const double r = m.info.resolution;
    const int i = static_cast<int>(std::floor((p.x - m.info.origin.position.x) / r));
    const int j = static_cast<int>(std::floor((p.y - m.info.origin.position.y) / r));
    if (i < 0 || j < 0 || i >= static_cast<int>(m.info.width)
            || j >= static_cast<int>(m.info.height)) {
        return false;
    }
    return m.data[static_cast<size_t>(j) * m.info.width + i] >= LETHAL;
}

}  // namespace

class ConstrainedPass : public rclcpp::Node
{
public:
    using Action = action::ConstrainedPass;
    using Handle = rclcpp_action::ServerGoalHandle<Action>;

    explicit ConstrainedPass(const rclcpp::NodeOptions &options)
        : Node("constrained_pass", options), tf_(get_clock()), tf_listener_(tf_)
    {
        base_frame_ = declare_parameter("robot_base_frame", std::string("base_link"));
        global_frame_ = declare_parameter("global_frame", std::string("map"));
        rate_ = declare_parameter("rate", 20.0);
        k_pos_ = declare_parameter("k_pos", 1.0);
        k_yaw_ = declare_parameter("k_yaw", 1.5);
        v_max_ = declare_parameter("correction_speed_max", 0.1);
        w_max_ = declare_parameter("correction_turn_max", 0.3);
        w_min_ = declare_parameter("turn_speed_min", 0.15);
        v_min_ = declare_parameter("align_speed_min", 0.03);
        timeout_ = declare_parameter("timeout", 60.0);
        const std::string footprint = declare_parameter("check_footprint", std::string(""));
        if (!nav2_costmap_2d::makeFootprintFromString(footprint, footprint_)
                || footprint_.size() < 3) {
            throw std::runtime_error("check_footprint '" + footprint + "' is not a polygon");
        }
        for (const auto &p : footprint_) half_length_ = std::max(half_length_, std::fabs(p.x));
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                declare_parameter("costmap_topic", std::string("local_costmap/costmap")), 1,
                [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
                    const std::lock_guard<std::mutex> lock(mtx_);
                    costmap_ = m;
                });
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
                declare_parameter("cmd_vel_topic", std::string("cmd_vel_nav")), 10);
        server_ = rclcpp_action::create_server<Action>(
                this, "constrained_pass",
                [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                },
                [](std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
                [this](std::shared_ptr<Handle> h) { std::thread([this, h] { execute(h); }).detach(); });
    }

private:
    struct Pose
    {
        Vec p;
        double yaw;
    };

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap()
    {
        const std::lock_guard<std::mutex> lock(mtx_);
        return costmap_;
    }

    std::optional<Pose> lookup(const std::string &target, const std::string &source)
    {
        try {
            const auto t = tf_.lookupTransform(target, source, tf2::TimePointZero).transform;
            return Pose{ { t.translation.x, t.translation.y }, tf2::getYaw(t.rotation) };
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN(get_logger(), "%s", e.what());
            return std::nullopt;
        }
    }

    struct Gap
    {
        Vec centre;
        double width;
    };

    // narrowest lethal-cell gap along the wall; +-0.15 m reaches the faces seen when side-on
    std::optional<Gap> gap(const nav_msgs::msg::OccupancyGrid &m, Vec centre, Vec dir, double width)
    {
        const double r = m.info.resolution;
        const Vec along = dir.perp();
        const double reach = width / 2 + 0.2;
        double lo = -INFINITY, hi = INFINITY;
        for (double depth = -0.15; depth <= 0.15 + 1e-9; depth += r) {
            for (double k = -reach; k <= reach; k += r / 2) {
                if (lethal(m, centre + dir * depth + along * k)) {
                    if (k < 0) lo = std::max(lo, k);
                    else hi = std::min(hi, k);
                }
            }
        }
        if (!std::isfinite(lo) || !std::isfinite(hi)) return std::nullopt;
        return Gap{ centre + along * ((lo + hi) / 2), hi - lo - r };
    }

    // the first lethal cell on the outline, if any; with `motion`, only the edges facing it
    std::optional<Vec> footprint_hit(const nav_msgs::msg::OccupancyGrid &m, Pose at,
            const Vec *motion = nullptr, const std::function<bool(Vec)> &ignore = {})
    {
        const double c = std::cos(at.yaw), s = std::sin(at.yaw), step = m.info.resolution / 2;
        for (size_t i = 0; i < footprint_.size(); i++) {
            const auto &a = footprint_[i], &b = footprint_[(i + 1) % footprint_.size()];
            const Vec pa = at.p + Vec{ c * a.x - s * a.y, s * a.x + c * a.y };
            const Vec pb = at.p + Vec{ c * b.x - s * b.y, s * b.x + c * b.y };
            if (motion) {
                Vec out = (pb - pa).perp();
                if (out.dot((pa + pb) * 0.5 - at.p) < 0) out = out * -1.0;
                if (out.dot(*motion) <= 0) continue;
            }
            const int n = std::max(1, static_cast<int>(std::ceil((pb - pa).norm() / step)));
            for (int k = 0; k <= n; k++) {
                const Vec q = pa + (pb - pa) * (static_cast<double>(k) / n);
                if (!(ignore && ignore(q)) && lethal(m, q)) return q;
            }
        }
        return std::nullopt;
    }

    void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

    void finish(const std::shared_ptr<Handle> &h, uint16_t code, const std::string &msg)
    {
        stop();
        auto result = std::make_shared<Action::Result>();
        result->error_code = code;
        result->error_msg = msg;
        if (code == Action::Result::NONE) {
            RCLCPP_INFO(get_logger(), "passed %s", h->get_goal()->name.c_str());
            h->succeed(result);
        } else {
            RCLCPP_WARN(get_logger(), "%s: %s", h->get_goal()->name.c_str(), msg.c_str());
            h->abort(result);
        }
    }

    void execute(const std::shared_ptr<Handle> &h)
    {
        const auto goal = h->get_goal();
        const auto &cons = goal->constraints;
        const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_);

        auto m = costmap();
        for (int i = 0; !m && i < 20; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            m = costmap();
        }
        if (!m) return finish(h, Action::Result::TF_ERROR, "no costmap");
        const std::string frame = m->header.frame_id;

        // the zone in the costmap's frame (odom: smooth, unlike map)
        const auto map_in_frame = lookup(frame, global_frame_);
        if (!map_in_frame) return finish(h, Action::Result::TF_ERROR, "no " + global_frame_);
        const double c = std::cos(map_in_frame->yaw), s = std::sin(map_in_frame->yaw);
        const Vec bim = map_in_frame->p
                + Vec{ c * goal->centre.x - s * goal->centre.y,
                       s * goal->centre.x + c * goal->centre.y };
        const Vec dir = Vec{ c * goal->direction.x - s * goal->direction.y,
                             s * goal->direction.x + c * goal->direction.y }
                * (1.0 / std::hypot(goal->direction.x, goal->direction.y));
        const Vec normal = dir.perp();

        const auto start = lookup(frame, base_frame_);
        if (!start) return finish(h, Action::Result::TF_ERROR, "no " + base_frame_);
        // in the doorway the jambs are beside the robot, not a gap ahead of it
        const bool inside = std::fabs((start->p - bim).dot(dir)) < half_length_ + 0.1;

        Vec centre = bim + normal * last_offset_[goal->name];
        double gap_half = goal->width / 2;
        if (auto seen = gap(*m, bim, dir, goal->width)) {
            centre = seen->centre;
            gap_half = seen->width / 2;
            last_offset_[goal->name] = (centre - bim).dot(normal);
            RCLCPP_INFO(get_logger(), "%s: lidar gap %.3f m, centre %+.3f m from the zone",
                    goal->name.c_str(), seen->width, last_offset_[goal->name]);
        } else if (!inside) {
            return finish(h, Action::Result::NO_GAP, "the lidars see no gap");
        }
        const double through = std::atan2(dir.y, dir.x);
        double heading = start->yaw;
        if (cons.heading_mode == msg::MotionConstraints::HEADING_FORWARD) {
            heading = through;
        } else if (cons.heading_mode == msg::MotionConstraints::HEADING_ALONG) {
            heading = std::fabs(wrap(start->yaw - through)) <= M_PI / 2 ? through : through + M_PI;
        }

        // turn on the spot, then slide into line with the heading held, then drive through;
        // in the doorway there is no room to turn, so carry on through
        enum class Phase { TURN, ALIGN, PASS } phase = inside ? Phase::PASS : Phase::TURN;
        if (inside) {
            heading = std::fabs(wrap(start->yaw - through)) <= M_PI / 2 ? through : through + M_PI;
            RCLCPP_INFO(get_logger(), "%s: in the doorway, passing", goal->name.c_str());
        }
        auto feedback = std::make_shared<Action::Feedback>();
        rclcpp::Rate rate(rate_);
        while (rclcpp::ok()) {
            if (h->is_canceling()) {
                stop();
                h->canceled(std::make_shared<Action::Result>());
                return;
            }
            if (now() > deadline) return finish(h, Action::Result::TIMEOUT, "timed out");

            // follow the gap as the costmap updates
            if (auto latest = costmap(); latest != m) {
                m = latest;
                if (auto again = gap(*m, bim, dir, goal->width)) {
                    centre = again->centre;
                    gap_half = again->width / 2;
                }
            }

            const auto base = lookup(frame, base_frame_);
            if (!base) return finish(h, Action::Result::TF_ERROR, "no " + base_frame_);
            const Vec p = base->p - centre;
            const double along = p.dot(dir), lateral = p.dot(normal);
            const double yaw_err = wrap(base->yaw - heading);

            if (phase == Phase::TURN && std::fabs(yaw_err) < cons.align_tolerance_yaw) {
                phase = Phase::ALIGN;
                RCLCPP_INFO(get_logger(), "%s: turned, aligning", goal->name.c_str());
            }
            if (phase == Phase::ALIGN && std::fabs(lateral) < cons.align_tolerance_xy) {
                phase = Phase::PASS;
                RCLCPP_INFO(get_logger(), "%s: aligned, passing", goal->name.c_str());
            }
            if (phase == Phase::PASS && along >= goal->approach) {
                return finish(h, Action::Result::NONE, "");
            }

            Vec v;
            if (phase == Phase::ALIGN) {
                // like the turn: a few cm/s at least, or the base creeps the last centimetres
                const double slide = std::max(std::fabs(k_pos_ * lateral), v_min_);
                v = dir * clip(k_pos_ * (-goal->approach - along), v_max_)
                        - normal * std::copysign(std::min(slide, v_max_), lateral);
            } else if (phase == Phase::PASS) {
                v = dir * cons.max_speed - normal * clip(k_pos_ * lateral, v_max_);
            }
            double w = clip(-k_yaw_ * yaw_err, w_max_);
            if (phase == Phase::TURN) {
                // below this the base barely turns: castor friction
                w = std::copysign(std::max(std::fabs(w), w_min_), w);
            }

            // the measured jambs (and their lidar jitter) are the door, not an obstacle
            const double jitter = 2 * m->info.resolution;
            const auto jamb = [&](Vec q) {
                const Vec d = q - centre;
                // 0.35 m through: jamb faces of walls up to ~0.5 m thick
                return std::fabs(d.dot(dir)) < 0.35 && std::fabs(d.dot(normal)) > gap_half - jitter;
            };
            const Pose ahead{ base->p + v * cons.stop_time, base->yaw + w * cons.stop_time };
            const auto hit = phase == Phase::PASS ? footprint_hit(*m, ahead, &v, jamb)
                                                  : footprint_hit(*m, ahead);
            if (hit) {
                const Vec d = *hit - centre;
                char where[96];
                std::snprintf(where, sizeof(where), " (%+.2f m through, %+.2f m across the gap)",
                        d.dot(dir), d.dot(normal));
                return finish(h, Action::Result::OBSTACLE, "obstacle within "
                        + std::to_string(cons.stop_time) + " s ahead" + where);
            }

            geometry_msgs::msg::Twist cmd;
            const double cy = std::cos(base->yaw), sy = std::sin(base->yaw);
            cmd.linear.x = cy * v.x + sy * v.y;
            cmd.linear.y = -sy * v.x + cy * v.y;
            cmd.angular.z = w;
            cmd_pub_->publish(cmd);

            feedback->phase = phase == Phase::TURN ? "turn" : phase == Phase::ALIGN ? "align" : "pass";
            feedback->lateral_error = lateral;
            feedback->heading_error = yaw_err;
            h->publish_feedback(feedback);
            rate.sleep();
        }
        stop();
    }

    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener tf_listener_;
    std::string base_frame_, global_frame_;
    double rate_, k_pos_, k_yaw_, v_max_, v_min_, w_max_, w_min_, timeout_;
    double half_length_ = 0;
    std::map<std::string, double> last_offset_;  // per zone: the gap's offset along its wall
    std::vector<geometry_msgs::msg::Point> footprint_;
    std::mutex mtx_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace eddie_navigation

RCLCPP_COMPONENTS_REGISTER_NODE(eddie_navigation::ConstrainedPass)
