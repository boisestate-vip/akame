#include <memory>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Float64;

enum class RobotState {
  SEARCH_FOR_DIG_ZONE = 0,
  NAV_TO_DIG = 1,
  DIG = 2,
  SEARCH_FOR_DUMP_ZONE = 3,
  NAV_TO_DUMP = 4,
  DUMP = 5
};

class CentralController : public rclcpp::Node {
public:
  CentralController()
  : Node("central_controller"),
    map_(nullptr),
    pose_(nullptr),
    dig_zone_known_(false),
    dump_zone_known_(false),
    state_(RobotState::SEARCH_FOR_DIG_ZONE)
  {
    map_sub_ = this->create_subscription<OccupancyGrid>(
      "map", 10,
      std::bind(&CentralController::map_callback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<PoseStamped>(
      "pose_in", 10,
      std::bind(&CentralController::pose_callback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<PoseStamped>("goal_pose", 10);
    arm_pub_ = this->create_publisher<Float64>("regolith_arm_angle", 10);
    drum_pub_ = this->create_publisher<Float64>("regolith_drum_speed", 10);
  }

private:
  // subscribers & publishers
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<Float64>::SharedPtr arm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr drum_pub_;

  // internal state
  std::shared_ptr<OccupancyGrid> map_;
  std::shared_ptr<PoseStamped> pose_;
  bool dig_zone_known_;
  bool dump_zone_known_;
  std::pair<double,double> dig_zone_;
  std::pair<double,double> dump_zone_;
  RobotState state_;

  // ===========================
  // callbacks
  // ===========================
  void map_callback(const OccupancyGrid::SharedPtr msg) {
    map_ = msg;
    // placeholder: detect dig/dump zones from map if needed
  }

  void pose_callback(const PoseStamped::SharedPtr msg) {
    pose_ = msg;
    update_state_machine();
  }

  // ===========================
  // state machine
  // ===========================
  void update_state_machine() {
    if (!pose_) return;

    double x = pose_->pose.position.x;
    double y = pose_->pose.position.y;

    if (state_ == RobotState::SEARCH_FOR_DIG_ZONE) {
      RCLCPP_INFO(this->get_logger(), "→ NAV_TO_DIG");
      // If no dig zone is known, choose a random goal (simple exploration).
      if (!dig_zone_known_) {
        auto [gx, gy] = random_search_goal();
        dig_zone_ = {gx, gy};
        dig_zone_known_ = true;
      }
      state_ = RobotState::NAV_TO_DIG;
      return;
    }

    if (state_ == RobotState::NAV_TO_DIG) {
      double dig_x = dig_zone_.first;
      double dig_y = dig_zone_.second;
      if (is_near(x, y, dig_x, dig_y)) {
        RCLCPP_INFO(this->get_logger(), "→ DIG");
        state_ = RobotState::DIG;
      } else {
        publish_goal(dig_x, dig_y);
      }
      return;
    }

    if (state_ == RobotState::DIG) {
      publish_arm_commands();
      RCLCPP_INFO(this->get_logger(), "→ SEARCH_FOR_DUMP_ZONE");
      state_ = RobotState::SEARCH_FOR_DUMP_ZONE;
      dig_zone_known_ = false;
      return;
    }

    if (state_ == RobotState::SEARCH_FOR_DUMP_ZONE) {
      RCLCPP_INFO(this->get_logger(), "→ NAV_TO_DUMP");
      if (!dump_zone_known_) {
        auto [gx, gy] = random_search_goal();
        dump_zone_ = {gx, gy};
        dump_zone_known_ = true;
      }
      state_ = RobotState::NAV_TO_DUMP;
      return;
    }

    if (state_ == RobotState::NAV_TO_DUMP) {
      double dump_x = dump_zone_.first;
      double dump_y = dump_zone_.second;
      if (is_near(x, y, dump_x, dump_y)) {
        RCLCPP_INFO(this->get_logger(), "→ DUMP");
        state_ = RobotState::DUMP;
      } else {
        publish_goal(dump_x, dump_y);
      }
      return;
    }

    if (state_ == RobotState::DUMP) {
      publish_arm_commands();
      RCLCPP_INFO(this->get_logger(), "→ SEARCH_FOR_DIG_ZONE");
      state_ = RobotState::SEARCH_FOR_DIG_ZONE;
      dump_zone_known_ = false;
      return;
    }
  }

  // ===========================
  // helpers
  // ===========================
  bool is_near(double x, double y, double gx, double gy, double tol = 0.3) {
    return std::abs(x - gx) < tol && std::abs(y - gy) < tol;
  }

  void publish_goal(double x, double y) {
    PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal_pub_->publish(goal);
  }

  void publish_arm_commands() {
    Float64 arm;
    Float64 drum;
    // lower arm
    arm.data = -0.5;
    drum.data = (state_ == RobotState::DIG) ? 3.0 : -3.0;
    arm_pub_->publish(arm);
    drum_pub_->publish(drum);
    // raise arm
    arm.data = 1.0;
    arm_pub_->publish(arm);
  }

  std::pair<double,double> random_search_goal() {
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<double> dist(-5.0, 5.0);
    return {dist(rng), dist(rng)};
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CentralController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
