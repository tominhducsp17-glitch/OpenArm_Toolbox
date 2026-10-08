// Node 2 "controller": calls the openarm_control library (control-toolbox interface) and outputs tau_ff.
//
//   in : joint_states   (sensor_msgs/JointState)  measured position/velocity (+ finger positions)
//   in : joint_commands (sensor_msgs/JointState)  position = q_target, velocity = dq_target
//   out: controller/tau_ff (sensor_msgs/JointState) effort = tau_ff [N m] for the joints of the command,
//        stamped with the command's stamp
//
// One output per received command (event driven). ddq_target is estimated from dq_target. Joints of
// joint_names that a command does not contain (e.g. the other arm) are held at their measured position
// with zero velocity inside the model and get no output. The controller is chosen by controller_type
// through openarm_control::makeTrackingController; the node only uses ct::core::Controller::computeControl.
#include <openarm_control/tracking_controller.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using openarm_control::JointVector;

template<size_t NJ>
class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const std::vector<std::string> & joints)
  : Node("controller"), joints_(joints), estimator_(
      declare_parameter<double>("acceleration_cutoff_hz", 5.0),
      declare_parameter<double>("max_abs_acceleration", 20.0))
  {
    declare_parameter<std::vector<std::string>>("joint_names", joints_);
    declare_parameter<std::string>("urdf_path", "");
    declare_parameter<std::string>("controller_type", "ctc_feedforward");
    const std::vector<double> one_arm{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0};
    std::vector<double> default_limit;
    for (std::size_t k = 0; k < NJ; ++k) {default_limit.push_back(one_arm[k % 7]);}
    declare_parameter<std::vector<double>>("torque_limit", default_limit);
    declare_parameter<double>("input_timeout_s", 0.1);
    declare_parameter<std::string>("state_topic", "joint_states");
    declare_parameter<std::string>("command_topic", "joint_commands");
    declare_parameter<std::string>("output_topic", "controller/tau_ff");
    openarm_control::TrackingControllerConfig config;
    config.urdf_path = get_parameter("urdf_path").as_string();
    if (config.urdf_path.empty()) {
      config.urdf_path = ament_index_cpp::get_package_share_directory("openarm_control") + "/urdf/openarm_v1_bimanual.urdf";
    }
    config.joint_names = joints_;
    config.torque_limit = get_parameter("torque_limit").as_double_array();
    const auto type = get_parameter("controller_type").as_string();
    controller_ = openarm_control::makeTrackingController<NJ>(type, config);
    timeout_s_ = get_parameter("input_timeout_s").as_double();
    for (std::size_t j = 0; j < NJ; ++j) {index_[joints_[j]] = j;}
    q_meas_.setZero();
    dq_meas_.setZero();

    tau_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("output_topic").as_string(), rclcpp::QoS(10));
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("state_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & m) {onState(m);});
    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("command_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & m) {onCommand(m);});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(10), [this] {onWatchdog();});
    RCLCPP_INFO(get_logger(), "controller '%s' (ct::core::Controller<%zu, %zu>), %zu joints, URDF %s",
      controller_->name().c_str(), 2 * NJ, NJ, NJ, config.urdf_path.c_str());
  }

private:
  void onState(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {return;}
    for (std::size_t k = 0; k < msg.name.size(); ++k) {
      const auto it = index_.find(msg.name[k]);
      if (it == index_.end()) {
        controller_->setPassiveJointPosition(msg.name[k], msg.position[k]);  // fingers
        continue;
      }
      if (!std::isfinite(msg.position[k])) {continue;}
      q_meas_[it->second] = msg.position[k];
      dq_meas_[it->second] = msg.velocity.size() == msg.name.size() ? msg.velocity[k] : 0.0;
      measured_[it->second] = true;
    }
  }

  void onCommand(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_commands without positions");
      return;
    }
    const bool has_velocity = msg.velocity.size() == msg.name.size();
    if (!has_velocity) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_commands without velocity; dq_target = 0");
    }
    openarm_control::JointReference<NJ> ref;
    ref.q = q_meas_;  // joints without a command hold their measured pose
    std::vector<std::size_t> commanded;
    for (std::size_t k = 0; k < msg.name.size(); ++k) {
      const auto it = index_.find(msg.name[k]);
      if (it == index_.end()) {continue;}
      ref.q[it->second] = msg.position[k];
      ref.dq[it->second] = has_velocity ? msg.velocity[k] : 0.0;
      commanded.push_back(it->second);
    }
    if (commanded.empty()) {return;}
    for (std::size_t j = 0; j < NJ; ++j) {
      const bool in_command = std::find(commanded.begin(), commanded.end(), j) != commanded.end();
      if (!in_command && !measured_[j]) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "%s has neither a command nor a measurement yet; tau_ff not computed", joints_[j].c_str());
        return;
      }
    }
    const bool stamped = msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0;
    const double t = stamped ? rclcpp::Time(msg.header.stamp).seconds() : now().seconds();
    ref.ddq = estimator_.update(t, ref.dq);
    try {
      controller_->setReference(ref);
      openarm_control::State<NJ> x;
      x << q_meas_, dq_meas_;
      openarm_control::Torque<NJ> u;
      controller_->computeControl(x, t, u);  // ct::core::Controller interface
      sensor_msgs::msg::JointState out;
      out.header.stamp = stamped ? msg.header.stamp : builtin_interfaces::msg::Time(now());
      for (const auto j : commanded) {
        out.name.push_back(joints_[j]);
        out.effort.push_back(u[j]);
      }
      tau_pub_->publish(out);
      last_command_ = now();
      timed_out_ = false;
      if (controller_->lastClamped()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tau_ff clamped by torque_limit");
      }
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "command rejected: %s", e.what());
    }
  }

  void onWatchdog()
  {
    if (!last_command_ || timed_out_ || (now() - *last_command_).seconds() <= timeout_s_) {return;}
    timed_out_ = true;
    estimator_.reset();
    RCLCPP_WARN(get_logger(), "no joint_commands for %.3f s", timeout_s_);
  }

  std::vector<std::string> joints_;
  std::map<std::string, std::size_t> index_;
  std::unique_ptr<openarm_control::JointTrackingController<NJ>> controller_;
  openarm_control::AccelerationEstimator<NJ> estimator_;
  JointVector<NJ> q_meas_, dq_meas_;
  std::array<bool, NJ> measured_{};
  double timeout_s_;
  std::optional<rclcpp::Time> last_command_;
  bool timed_out_ = false;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tau_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_, command_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

std::vector<std::string> jointNamesParameter()
{
  // Short-lived node with the same name, so YAML/launch overrides for "controller" apply to it.
  std::vector<std::string> bimanual;
  for (const char * side : {"left", "right"}) {
    for (int j = 1; j <= 7; ++j) {bimanual.push_back(std::string("openarm_") + side + "_joint" + std::to_string(j));}
  }
  auto probe = std::make_shared<rclcpp::Node>("controller");
  return probe->declare_parameter<std::vector<std::string>>("joint_names", bimanual);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    const auto joints = jointNamesParameter();
    std::shared_ptr<rclcpp::Node> node;
    if (joints.size() == 7) {
      node = std::make_shared<ControllerNode<7>>(joints);
    } else if (joints.size() == 14) {
      node = std::make_shared<ControllerNode<14>>(joints);
    } else {
      throw std::invalid_argument("joint_names must list 7 (one arm) or 14 (both arms) joints");
    }
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("controller"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
