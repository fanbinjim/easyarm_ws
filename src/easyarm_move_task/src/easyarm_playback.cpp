#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "controller_mode_utils.hpp"

namespace
{

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;
using JointPositions = std::array<double, 6>;

const std::array<std::string, 6> kJointNames = {
  "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
};

struct Sample
{
  double t{0.0};
  JointPositions joints{};
};

struct Record
{
  std::vector<std::string> joint_names;
  std::vector<Sample> samples;
};

enum class Key
{
  None,
  Space,
  Left,
  Right,
  Quit,
};

enum class PlayerState
{
  Paused,
  Playing,
  Moving,
  Finished,
};

class RawTerminal
{
public:
  RawTerminal()
  {
    if (!isatty(STDIN_FILENO)) {
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_) != 0) {
      return;
    }

    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      enabled_ = true;
    }
  }

  ~RawTerminal()
  {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

  bool enabled() const
  {
    return enabled_;
  }

  Key read_key() const
  {
    const int c = read_byte();
    if (c < 0) {
      return Key::None;
    }

    if (c == ' ') {
      return Key::Space;
    }
    if (c == 'q' || c == 'Q') {
      return Key::Quit;
    }
    if (c != 27) {
      return Key::None;
    }

    const int bracket = read_byte(std::chrono::milliseconds(20));
    const int code = read_byte(std::chrono::milliseconds(20));
    if (bracket == '[' && code == 'C') {
      return Key::Right;
    }
    if (bracket == '[' && code == 'D') {
      return Key::Left;
    }
    return Key::None;
  }

private:
  int read_byte(std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) const
  {
    if (!enabled_) {
      return -1;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);

    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);

    const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &tv);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
      return -1;
    }

    unsigned char c = 0;
    if (read(STDIN_FILENO, &c, 1) != 1) {
      return -1;
    }
    return c;
  }

  termios original_{};
  bool enabled_{false};
};

class JointStateCache
{
public:
  void update(const sensor_msgs::msg::JointState & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      for (size_t j = 0; j < msg.name.size(); ++j) {
        if (msg.name[j] == kJointNames[i] && j < msg.position.size()) {
          positions_[i] = msg.position[j];
          received_[i] = true;
          break;
        }
      }
    }
  }

  bool get(JointPositions & positions) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (bool received : received_) {
      if (!received) {
        return false;
      }
    }
    positions = positions_;
    return true;
  }

private:
  mutable std::mutex mutex_;
  JointPositions positions_{};
  std::array<bool, 6> received_{};
};

std::string join_joint_names(const std::vector<std::string> & names)
{
  std::ostringstream oss;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << names[i];
  }
  return oss.str();
}

bool is_finite(double value)
{
  return std::isfinite(value);
}

Record load_record(const std::string & path)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open record file: " + path);
  }

  nlohmann::json data;
  in >> data;

  Record record;
  record.joint_names = data.at("joint_names").get<std::vector<std::string>>();

  const auto & samples = data.at("samples");
  if (!samples.is_array()) {
    throw std::runtime_error("JSON field 'samples' must be an array");
  }

  for (const auto & sample : samples) {
    Sample parsed;
    parsed.t = sample.at("t").get<double>();
    const auto joints = sample.at("joints").get<std::vector<double>>();
    if (joints.size() != kJointNames.size()) {
      throw std::runtime_error("Sample joints size is not 6");
    }
    std::copy(joints.begin(), joints.end(), parsed.joints.begin());
    record.samples.push_back(parsed);
  }

  return record;
}

void validate_record(const Record & record, double max_playback_velocity, double speed_scale)
{
  if (record.joint_names.size() != kJointNames.size()) {
    throw std::runtime_error("joint_names size is not 6");
  }
  for (size_t i = 0; i < kJointNames.size(); ++i) {
    if (record.joint_names[i] != kJointNames[i]) {
      throw std::runtime_error(
        "Unexpected joint_names order: " + join_joint_names(record.joint_names));
    }
  }
  if (record.samples.empty()) {
    throw std::runtime_error("Record contains no samples");
  }

  for (size_t i = 0; i < record.samples.size(); ++i) {
    if (!is_finite(record.samples[i].t) || record.samples[i].t < 0.0) {
      throw std::runtime_error("Invalid sample timestamp");
    }
    for (double joint : record.samples[i].joints) {
      if (!is_finite(joint)) {
        throw std::runtime_error("Invalid joint value");
      }
    }

    if (i == 0) {
      continue;
    }

    const double dt = record.samples[i].t - record.samples[i - 1].t;
    if (!is_finite(dt) || dt <= 0.0) {
      throw std::runtime_error("Sample timestamps must be strictly increasing");
    }

    for (size_t j = 0; j < kJointNames.size(); ++j) {
      const double raw_velocity = std::abs(
        record.samples[i].joints[j] - record.samples[i - 1].joints[j]) / dt;
      const double velocity = raw_velocity * speed_scale;
      if (velocity > max_playback_velocity) {
        std::ostringstream oss;
        oss << "Playback velocity too high at sample " << i << ", " << kJointNames[j]
            << ": raw " << raw_velocity << " rad/s, speed_scale " << speed_scale
            << ", actual " << velocity << " rad/s > "
            << max_playback_velocity << " rad/s";
        throw std::runtime_error(oss.str());
      }
    }
  }
}

bool wait_for_joint_positions(
  const JointStateCache & cache, JointPositions & positions,
  std::chrono::seconds timeout = std::chrono::seconds(3))
{
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    if (cache.get(positions)) {
      return true;
    }
    if (std::chrono::steady_clock::now() - start > timeout) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

double max_joint_delta(const JointPositions & a, const JointPositions & b)
{
  double max_delta = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    max_delta = std::max(max_delta, std::abs(a[i] - b[i]));
  }
  return max_delta;
}

rclcpp::Duration seconds_to_duration(double seconds)
{
  return rclcpp::Duration::from_seconds(seconds);
}

trajectory_msgs::msg::JointTrajectory make_point_trajectory(
  const JointPositions & positions, double time_from_start)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names.assign(kJointNames.begin(), kJointNames.end());

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.assign(positions.begin(), positions.end());
  point.velocities = std::vector<double>(kJointNames.size(), 0.0);
  point.time_from_start = seconds_to_duration(time_from_start);
  trajectory.points.push_back(point);
  return trajectory;
}

trajectory_msgs::msg::JointTrajectory make_move_trajectory(
  const JointPositions & from, const JointPositions & to, double duration)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names.assign(kJointNames.begin(), kJointNames.end());

  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions.assign(from.begin(), from.end());
  start.velocities = std::vector<double>(kJointNames.size(), 0.0);
  start.time_from_start = seconds_to_duration(0.2);
  trajectory.points.push_back(start);

  trajectory_msgs::msg::JointTrajectoryPoint end;
  end.positions.assign(to.begin(), to.end());
  end.velocities = std::vector<double>(kJointNames.size(), 0.0);
  end.time_from_start = seconds_to_duration(duration);
  trajectory.points.push_back(end);

  return trajectory;
}

class TrajectoryClient
{
public:
  explicit TrajectoryClient(rclcpp::Node::SharedPtr node)
  : node_(std::move(node))
  {
    client_ = rclcpp_action::create_client<FollowJT>(
      node_->get_node_base_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
      "arm_controller/follow_joint_trajectory");
  }

  bool wait_for_server(std::chrono::seconds timeout)
  {
    return client_->wait_for_action_server(timeout);
  }

  bool send_and_wait(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::chrono::milliseconds timeout_margin = std::chrono::milliseconds(1500))
  {
    auto result_future = send(trajectory);
    if (!result_future.valid()) {
      return false;
    }

    const auto timeout = trajectory_duration(trajectory) + timeout_margin;
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      cancel_active();
      return false;
    }
    return result_future.get() == rclcpp_action::ResultCode::SUCCEEDED;
  }

  std::shared_future<rclcpp_action::ResultCode> send(
    const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    auto result_promise = std::make_shared<std::promise<rclcpp_action::ResultCode>>();
    auto result_future = result_promise->get_future().share();

    FollowJT::Goal goal;
    goal.trajectory = trajectory;

    auto options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    options.goal_response_callback =
      [this, result_promise](GoalHandleFollowJT::SharedPtr goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_goal_ = goal_handle;
        if (!goal_handle) {
          result_promise->set_value(rclcpp_action::ResultCode::UNKNOWN);
        }
      };
    options.result_callback =
      [this, result_promise](const GoalHandleFollowJT::WrappedResult & result) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_goal_.reset();
        }
        result_promise->set_value(result.code);
      };

    client_->async_send_goal(goal, options);
    return result_future;
  }

  void cancel_active()
  {
    GoalHandleFollowJT::SharedPtr goal_handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle = active_goal_;
    }
    if (goal_handle) {
      client_->async_cancel_goal(goal_handle);
    }
  }

private:
  static std::chrono::milliseconds trajectory_duration(
    const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    if (trajectory.points.empty()) {
      return std::chrono::milliseconds(0);
    }
    const auto & duration = trajectory.points.back().time_from_start;
    const int64_t milliseconds =
      static_cast<int64_t>(duration.sec) * 1000 + duration.nanosec / 1000000;
    return std::chrono::milliseconds(milliseconds);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJT>::SharedPtr client_;
  std::mutex mutex_;
  GoalHandleFollowJT::SharedPtr active_goal_;
};

trajectory_msgs::msg::JointTrajectory make_playback_trajectory(
  const Record & record, size_t start_index, double speed_scale,
  double start_delay)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names.assign(kJointNames.begin(), kJointNames.end());
  if (start_index >= record.samples.size()) {
    return trajectory;
  }

  const double t0 = record.samples[start_index].t;
  for (size_t index = start_index; index < record.samples.size(); ++index) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(record.samples[index].joints.begin(), record.samples[index].joints.end());
    const double t = start_delay + (record.samples[index].t - t0) / speed_scale;
    point.time_from_start = seconds_to_duration(t);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

size_t nearest_sample_index(const Record & record, const JointPositions & positions)
{
  size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < record.samples.size(); ++i) {
    double distance = 0.0;
    for (size_t j = 0; j < kJointNames.size(); ++j) {
      const double delta = positions[j] - record.samples[i].joints[j];
      distance += delta * delta;
    }
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

void hold_current_position(
  const JointStateCache & cache, TrajectoryClient & client,
  rclcpp::Logger logger)
{
  JointPositions current{};
  if (!wait_for_joint_positions(cache, current)) {
    RCLCPP_WARN(logger, "Failed to read current joint states for hold trajectory");
    return;
  }
  if (!client.send_and_wait(make_point_trajectory(current, 0.2))) {
    RCLCPP_WARN(logger, "Failed to send hold trajectory");
  }
}

std::string state_name(PlayerState state)
{
  switch (state) {
    case PlayerState::Paused:
      return "PAUSED";
    case PlayerState::Playing:
      return "PLAYING";
    case PlayerState::Moving:
      return "MOVING";
    case PlayerState::Finished:
      return "FINISHED";
  }
  return "UNKNOWN";
}

void render_menu(
  const std::string & file_path, PlayerState state, size_t index,
  const Record & record, const std::string & status)
{
  const double current_t = record.samples[std::min(index, record.samples.size() - 1)].t;
  const double end_t = record.samples.back().t;

  std::cout << "\033[2J\033[H";
  std::cout << "\n";
  std::cout << "EasyArm Playback\n";
  std::cout << "File: " << file_path << "\n";
  std::cout << "State: " << state_name(state) << "\n";
  std::cout << "Index: " << index << " / " << (record.samples.size() - 1) << "\n";
  std::cout << "Time: " << current_t << " / " << end_t << " s\n";
  std::cout << "Status: " << status << "\n";
  std::cout << "Controls:\n";
  std::cout << "  SPACE  play / pause\n";
  std::cout << "  RIGHT  move to next point when paused\n";
  std::cout << "  LEFT   move to previous point when paused\n";
  std::cout << "  q      quit\n";
  std::cout.flush();
}

bool confirm_playback(const std::string & file_path, const Record & record)
{
  std::cout << "About to play trajectory on real hardware.\n";
  std::cout << "File: " << file_path << "\n";
  std::cout << "Samples: " << record.samples.size() << "\n";
  std::cout << "Duration: " << record.samples.back().t << " s\n";
  std::cout << "Type 'yes' to continue: ";
  std::string answer;
  std::getline(std::cin, answer);
  return answer == "yes";
}

double clamped_duration(double duration, double min_duration, double max_duration)
{
  return std::min(std::max(duration, min_duration), max_duration);
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() != 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger("easyarm_playback"),
      "Usage: easyarm_playback <record.json>");
    rclcpp::shutdown();
    return 1;
  }
  const std::string file_path = args[1];

  auto node = std::make_shared<rclcpp::Node>("easyarm_playback");
  auto logger = node->get_logger();

  const bool require_confirm = node->declare_parameter<bool>("require_confirm", true);
  const double speed_scale = node->declare_parameter<double>("speed_scale", 1.0);
  const double approach_velocity = node->declare_parameter<double>("approach_velocity", 1.0);
  const double step_velocity = node->declare_parameter<double>("step_velocity", 0.3);
  const double max_playback_velocity = node->declare_parameter<double>("max_playback_velocity", 6.0);
  const bool autorepeat = node->declare_parameter<bool>("autorepeat", false);
  const double playback_start_delay = node->declare_parameter<double>("playback_start_delay", 0.2);
  const double min_approach_duration = node->declare_parameter<double>("min_approach_duration", 2.0);
  const double max_approach_duration = node->declare_parameter<double>("max_approach_duration", 10.0);
  const double min_step_duration = node->declare_parameter<double>("min_step_duration", 0.15);

  if (speed_scale <= 0.0 || approach_velocity <= 0.0 || step_velocity <= 0.0 ||
    max_playback_velocity <= 0.0 || playback_start_delay < 0.0)
  {
    RCLCPP_ERROR(logger, "Playback parameters must be positive");
    rclcpp::shutdown();
    return 1;
  }

  Record record;
  try {
    record = load_record(file_path);
    validate_record(record, max_playback_velocity, speed_scale);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Failed to load record: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  if (require_confirm && !confirm_playback(file_path, record)) {
    RCLCPP_WARN(logger, "Playback cancelled by user");
    rclcpp::shutdown();
    return 1;
  }

  JointStateCache joint_state_cache;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    [&joint_state_cache](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      joint_state_cache.update(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  TrajectoryClient trajectory_client(node);
  if (!trajectory_client.wait_for_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(logger, "arm_controller action server not available");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  JointPositions current{};
  if (!wait_for_joint_positions(joint_state_cache, current)) {
    RCLCPP_ERROR(logger, "Timeout waiting for joint_states");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Sending hold trajectory before switching to POSITION");
  if (!trajectory_client.send_and_wait(make_point_trajectory(current, 0.2))) {
    RCLCPP_ERROR(logger, "Failed to send initial hold trajectory");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Switching hardware to POSITION mode");
  if (!set_controller_mode(*node, "POSITION")) {
    RCLCPP_ERROR(logger, "Failed to set controller_mode to POSITION");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  const JointPositions start = record.samples.front().joints;
  const double approach_duration = clamped_duration(
    max_joint_delta(current, start) / approach_velocity,
    min_approach_duration,
    max_approach_duration);
  RCLCPP_INFO(logger, "Moving to start point in %.2f s", approach_duration);
  if (!trajectory_client.send_and_wait(make_move_trajectory(current, start, approach_duration))) {
    RCLCPP_ERROR(logger, "Failed to move to start point");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  RawTerminal terminal;
  if (!terminal.enabled()) {
    RCLCPP_ERROR(logger, "stdin is not an interactive terminal");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  PlayerState state = PlayerState::Paused;
  size_t index = 0;
  std::shared_future<rclcpp_action::ResultCode> active_result;
  std::string status = "At start point. Press SPACE to play.";
  auto last_render = std::chrono::steady_clock::now() - std::chrono::seconds(1);

  rclcpp::WallRate loop_rate(50.0);
  while (rclcpp::ok()) {
    const Key key = terminal.read_key();

    if (key == Key::Quit) {
      trajectory_client.cancel_active();
      hold_current_position(joint_state_cache, trajectory_client, logger);
      std::cout << "\033[2J\033[H\nEasyArm Playback exited.\n";
      break;
    }

    if (key == Key::Space) {
      if (state == PlayerState::Playing) {
        trajectory_client.cancel_active();
        hold_current_position(joint_state_cache, trajectory_client, logger);
        state = PlayerState::Paused;
        active_result = std::shared_future<rclcpp_action::ResultCode>();
        JointPositions paused_positions{};
        if (wait_for_joint_positions(joint_state_cache, paused_positions, std::chrono::seconds(1))) {
          index = nearest_sample_index(record, paused_positions);
        }
        status = "Paused.";
      } else if (state == PlayerState::Paused || state == PlayerState::Finished) {
        if (index >= record.samples.size() - 1) {
          index = 0;
        }
        state = PlayerState::Playing;
        status = "Playing.";
      }
    }

    if ((key == Key::Right || key == Key::Left) &&
      (state == PlayerState::Paused || state == PlayerState::Finished))
    {
      const size_t next_index = key == Key::Right ?
        std::min(index + 1, record.samples.size() - 1) :
        (index == 0 ? 0 : index - 1);

      if (next_index != index) {
        JointPositions from{};
        if (wait_for_joint_positions(joint_state_cache, from, std::chrono::seconds(1))) {
          const JointPositions & to = record.samples[next_index].joints;
          const double duration = std::max(
            min_step_duration, max_joint_delta(from, to) / step_velocity);
          state = PlayerState::Moving;
          render_menu(file_path, state, index, record, "Moving one step...");
          if (trajectory_client.send_and_wait(make_move_trajectory(from, to, duration))) {
            index = next_index;
            status = "Step complete.";
          } else {
            status = "Step failed.";
          }
          state = PlayerState::Paused;
        } else {
          status = "Failed to read current joints for step.";
        }
      }
    }

    if (state == PlayerState::Playing) {
      if (!active_result.valid()) {
        if (index >= record.samples.size() - 1) {
          state = PlayerState::Finished;
          status = "Playback finished. Press SPACE to replay or q to quit.";
        } else {
          active_result = trajectory_client.send(
            make_playback_trajectory(record, index, speed_scale, playback_start_delay));
        }
      } else if (active_result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        const auto result = active_result.get();
        if (result == rclcpp_action::ResultCode::SUCCEEDED) {
          index = record.samples.size() - 1;
          if (autorepeat) {
            JointPositions from{};
            if (wait_for_joint_positions(joint_state_cache, from, std::chrono::seconds(1))) {
              const double duration = clamped_duration(
                max_joint_delta(from, record.samples.front().joints) / approach_velocity,
                min_approach_duration,
                max_approach_duration);
              state = PlayerState::Moving;
              render_menu(file_path, state, index, record, "Looping: moving back to start...");
              if (trajectory_client.send_and_wait(
                  make_move_trajectory(from, record.samples.front().joints, duration)))
              {
                index = 0;
                state = PlayerState::Playing;
                status = "Auto repeat: playing next loop.";
              } else {
                state = PlayerState::Paused;
                status = "Auto repeat failed while moving back to start.";
              }
            } else {
              state = PlayerState::Paused;
              status = "Auto repeat failed: could not read current joints.";
            }
          } else {
            state = PlayerState::Finished;
            status = "Playback finished. Press SPACE to replay or q to quit.";
            hold_current_position(joint_state_cache, trajectory_client, logger);
          }
        } else {
          state = PlayerState::Paused;
          status = "Trajectory failed or was cancelled.";
          hold_current_position(joint_state_cache, trajectory_client, logger);
        }
        active_result = std::shared_future<rclcpp_action::ResultCode>();
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_render > std::chrono::milliseconds(100)) {
      render_menu(file_path, state, index, record, status);
      last_render = now;
    }

    loop_rate.sleep();
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
