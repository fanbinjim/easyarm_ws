#include <array>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "controller_mode_utils.hpp"

namespace
{

using JointPositions = std::array<double, 6>;

const std::array<std::string, 6> kJointNames = {
  "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
};

struct Sample
{
  double t{0.0};
  JointPositions positions{};
  geometry_msgs::msg::TransformStamped end_effector_pose{};
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

  int read_key() const
  {
    if (!enabled_) {
      return -1;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
      return -1;
    }

    unsigned char c = 0;
    if (read(STDIN_FILENO, &c, 1) != 1) {
      return -1;
    }
    return c;
  }

private:
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

std::string default_output_path()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  const auto package_prefix = std::filesystem::path(
    ament_index_cpp::get_package_prefix("easyarm_move_task"));
  const auto workspace_root = package_prefix.parent_path().parent_path();

  std::ostringstream date_dir;
  date_dir << std::put_time(&local_time, "%Y%m%d");

  std::ostringstream file_name;
  file_name << std::put_time(&local_time, "%H-%M-%S") << ".json";

  const auto output_dir = workspace_root / "data" / date_dir.str();
  std::filesystem::create_directories(output_dir);

  std::ostringstream path;
  path << (output_dir / file_name.str()).string();
  return path.str();
}

bool write_json(const std::string & path, const std::vector<Sample> & samples)
{
  nlohmann::json data;
  data["joint_names"] = kJointNames;
  data["sample_rate_hz"] = 50.0;
  data["position_unit"] = "rad";
  data["translation_unit"] = "m";
  data["rotation_unit"] = "quaternion_xyzw";

  if (!samples.empty()) {
    data["base_frame"] = samples.front().end_effector_pose.header.frame_id;
    data["ee_frame"] = samples.front().end_effector_pose.child_frame_id;
  }

  data["samples"] = nlohmann::json::array();
  for (const auto & sample : samples) {
    const auto & transform = sample.end_effector_pose.transform;
    data["samples"].push_back({
      {"t", sample.t},
      {"joints", sample.positions},
      {"ee_pose", {
        {"translation", {
          transform.translation.x,
          transform.translation.y,
          transform.translation.z}},
        {"rotation", {
          transform.rotation.x,
          transform.rotation.y,
          transform.rotation.z,
          transform.rotation.w}}
      }}
    });
  }

  std::ofstream out(path);
  if (!out.is_open()) {
    return false;
  }
  out << data.dump(2) << '\n';
  return out.good();
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const std::string output_path = argc >= 2 ? argv[1] : default_output_path();

  auto node = std::make_shared<rclcpp::Node>("easyarm_record");
  auto logger = node->get_logger();

  const std::string base_frame = node->declare_parameter<std::string>("base_frame", "base_link");
  const std::string end_effector_frame = node->declare_parameter<std::string>(
    "end_effector_frame", "Link6");

  JointStateCache joint_state_cache;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    [&joint_state_cache](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      joint_state_cache.update(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  RCLCPP_INFO(logger, "Switching hardware to DRAG mode");
  if (!set_controller_mode(*node, "DRAG")) {
    RCLCPP_ERROR(logger, "Failed to set controller_mode to DRAG");
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "controller_mode set to DRAG");

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

  RCLCPP_INFO(logger, "Press SPACE to start recording. Press SPACE again to stop and save.");
  RCLCPP_INFO(logger, "Output file: %s", output_path.c_str());
  RCLCPP_INFO(
    logger, "Recording end-effector pose from %s to %s",
    base_frame.c_str(), end_effector_frame.c_str());

  std::vector<Sample> samples;
  bool recording = false;
  bool missing_joint_warned = false;
  bool missing_tf_warned = false;
  auto start_time = std::chrono::steady_clock::now();
  auto next_sample_time = start_time;
  constexpr auto sample_period = std::chrono::milliseconds(20);

  rclcpp::WallRate loop_rate(200.0);
  while (rclcpp::ok()) {
    const int key = terminal.read_key();
    if (key == ' ') {
      if (!recording) {
        JointPositions positions{};
        if (!joint_state_cache.get(positions)) {
          RCLCPP_WARN(logger, "Waiting for complete Joint1-Joint6 positions before recording");
          loop_rate.sleep();
          continue;
        }

        samples.clear();
        start_time = std::chrono::steady_clock::now();
        next_sample_time = start_time;
        recording = true;
        missing_joint_warned = false;
        missing_tf_warned = false;
        RCLCPP_INFO(logger, "Recording started at 50 Hz");
      } else {
        RCLCPP_INFO(logger, "Recording stopped");
        break;
      }
    }

    if (recording) {
      const auto now = std::chrono::steady_clock::now();
      while (now >= next_sample_time) {
        JointPositions positions{};
        if (joint_state_cache.get(positions)) {
          geometry_msgs::msg::TransformStamped end_effector_pose;
          try {
            end_effector_pose = tf_buffer.lookupTransform(
              base_frame, end_effector_frame, tf2::TimePointZero);
          } catch (const tf2::TransformException & ex) {
            if (!missing_tf_warned) {
              RCLCPP_WARN(
                logger, "Skipping samples until TF %s -> %s is available: %s",
                base_frame.c_str(), end_effector_frame.c_str(), ex.what());
              missing_tf_warned = true;
            }
            next_sample_time += sample_period;
            continue;
          }

          const std::chrono::duration<double> t = next_sample_time - start_time;
          samples.push_back(Sample{t.count(), positions, end_effector_pose});
        } else if (!missing_joint_warned) {
          RCLCPP_WARN(logger, "Skipping samples until complete Joint1-Joint6 positions are available");
          missing_joint_warned = true;
        }

        next_sample_time += sample_period;
      }
    }

    loop_rate.sleep();
  }

  bool ok = true;
  if (!samples.empty()) {
    ok = write_json(output_path, samples);
    if (ok) {
      RCLCPP_INFO(logger, "Saved %zu samples to %s", samples.size(), output_path.c_str());
    } else {
      RCLCPP_ERROR(logger, "Failed to write %s", output_path.c_str());
    }
  } else {
    RCLCPP_WARN(logger, "No samples recorded; JSON file was not written");
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
