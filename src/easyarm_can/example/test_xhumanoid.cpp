/**
 * @file test_xhumanoid.cpp
 * @brief XHumanoid CAN 2.0B/CAN FD 控制测试程序。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "easyarm_can/easyarm_can.hpp"
#include "easyarm_can/model_registry.hpp"

namespace
{

enum class ControlMode
{
  Hybrid,
  Position,
  Velocity
};

struct Options
{
  std::string can_interface{"can0"};
  std::string model{"xhumanoid_60h_100"};
  ControlMode mode{ControlMode::Hybrid};
  bool is_canfd{false};
  uint8_t motor_id{1};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double current_limit_a{1.0};
  double kp{10.0};
  double kd{5.0};
  double torque_nm{0.0};
  int cycles{1000};
  int period_ms{5};
  bool dryrun{false};
  bool list_models{false};
};

void printUsage(const char * program)
{
  std::cout
    << "Usage: " << program << " [options]\n"
    << "  --can <name>        CAN interface, default can0\n"
    << "  --canfd            Use XHumanoid CAN FD protocol, default false\n"
    << "  --id <id>           Motor CAN ID, default 1\n"
    << "  --model <name>      Motor model, default xhumanoid_60h_100\n"
    << "  --mode <mode>       Control mode: hybrid, position or velocity, default hybrid\n"
    << "  --list-models       List builtin motor models\n"
    << "  --pos <rad>         Target position, default 0\n"
    << "  --vel <rad/s>       Velocity command/limit, default 0\n"
    << "  --current-limit <A> Position/velocity mode current limit, default 1\n"
    << "  --kp <value>        XHumanoid KP value, default 0\n"
    << "  --kd <value>        XHumanoid KD value, default 0\n"
    << "  --torque <Nm>       Feedforward torque, default 0\n"
    << "  --cycles <n>        Command cycles, default 100\n"
    << "  --period-ms <ms>    Command period, default 5\n"
    << "  --dryrun            Print payload without opening CAN or sending frames\n"
    << "  --help              Show this help\n";
}

bool parseU8(const std::string & text, uint8_t & value)
{
  char * end = nullptr;
  const long parsed = std::strtol(text.c_str(), &end, 0);
  if (end == text.c_str() || *end != '\0' || parsed < 0 || parsed > 255) {
    return false;
  }
  value = static_cast<uint8_t>(parsed);
  return true;
}

uint32_t floatToUint(double x, double x_min, double x_max, unsigned bits)
{
  if (bits == 0 || bits >= 32 || x_max <= x_min) {
    return 0;
  }
  if (x < x_min) {
    x = x_min;
  } else if (x > x_max) {
    x = x_max;
  }
  return static_cast<uint32_t>((x - x_min) * static_cast<double>((1u << bits) - 1u) /
         (x_max - x_min));
}

uint32_t clampRoundToUint(double value, double min_value, double max_value)
{
  if (value < min_value) {
    value = min_value;
  } else if (value > max_value) {
    value = max_value;
  }
  return static_cast<uint32_t>(std::lround(value));
}

double radToDeg(double rad)
{
  return rad * 180.0 / 3.14159265358979323846;
}

double radPerSecToRpm(double rad_s)
{
  return rad_s * 60.0 / (2.0 * 3.14159265358979323846);
}

void printPayload(const char * label, const uint8_t * data, std::size_t size)
{
  std::cout << label;
  for (std::size_t i = 0; i < size; ++i) {
    std::cout << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(data[i]);
  }
  std::cout << std::dec << std::setfill(' ') << "\n";
}

void printCan20HybridPayload(const Options & options)
{
  const uint16_t kp = static_cast<uint16_t>(floatToUint(options.kp, 0.0, 2000.0, 12));
  const uint16_t kd = static_cast<uint16_t>(floatToUint(options.kd, 0.0, 300.0, 9));
  const uint16_t q = static_cast<uint16_t>(floatToUint(options.position_rad, -6.28, 6.28, 16));
  const uint16_t dq = static_cast<uint16_t>(floatToUint(options.velocity_rad_s, -21.0, 21.0, 12));
  const uint16_t tau = static_cast<uint16_t>(floatToUint(options.torque_nm, -300.0, 300.0, 12));
  const uint8_t data[8] = {
    static_cast<uint8_t>((kp >> 7) & 0x1Fu),
    static_cast<uint8_t>(((kp & 0x7Fu) << 1) | ((kd >> 8) & 0x01u)),
    static_cast<uint8_t>(kd & 0xFFu),
    static_cast<uint8_t>((q >> 8) & 0xFFu),
    static_cast<uint8_t>(q & 0xFFu),
    static_cast<uint8_t>((dq >> 4) & 0xFFu),
    static_cast<uint8_t>(((dq & 0x0Fu) << 4) | ((tau >> 8) & 0x0Fu)),
    static_cast<uint8_t>(tau & 0xFFu),
  };

  printPayload("Expected hybrid payload: ", data, sizeof(data));
}

void writeFloatBe(uint8_t * data, float value)
{
  union FloatBytes
  {
    float value;
    uint8_t bytes[4];
  } convert{};
  convert.value = value;
  data[0] = convert.bytes[3];
  data[1] = convert.bytes[2];
  data[2] = convert.bytes[1];
  data[3] = convert.bytes[0];
}

void printCanfdHybridPayload(const Options & options)
{
  const uint16_t kp = static_cast<uint16_t>(
    std::lround(std::min(std::max(options.kp, 0.0), 6553.5) * 10.0));
  const uint16_t kd = static_cast<uint16_t>(
    std::lround(std::min(std::max(options.kd, 0.0), 6553.5) * 10.0));
  const int16_t tau = static_cast<int16_t>(
    std::lround(std::min(std::max(options.torque_nm, -32768.0), 32767.0)));
  uint8_t data[16] = {};
  data[0] = 0x11;
  data[1] = static_cast<uint8_t>((kp >> 8) & 0xFFu);
  data[2] = static_cast<uint8_t>(kp & 0xFFu);
  data[3] = static_cast<uint8_t>((kd >> 8) & 0xFFu);
  data[4] = static_cast<uint8_t>(kd & 0xFFu);
  writeFloatBe(&data[5], static_cast<float>(options.position_rad));
  writeFloatBe(&data[9], static_cast<float>(options.velocity_rad_s));
  data[13] = static_cast<uint8_t>((static_cast<uint16_t>(tau) >> 8) & 0xFFu);
  data[14] = static_cast<uint8_t>(static_cast<uint16_t>(tau) & 0xFFu);
  data[15] = 0;

  printPayload("Expected CAN FD hybrid payload: ", data, sizeof(data));
}

void printCan20PositionPayload(const Options & options)
{
  constexpr uint8_t kReportMessage1 = 0x01;
  const uint16_t velocity = static_cast<uint16_t>(
    clampRoundToUint(radPerSecToRpm(options.velocity_rad_s) * 10.0, 0.0, 32767.0));
  const uint16_t current = static_cast<uint16_t>(
    clampRoundToUint(options.current_limit_a * 10.0, 0.0, 4095.0));

  uint8_t position[4] = {};
  writeFloatBe(position, static_cast<float>(radToDeg(options.position_rad)));

  const uint8_t data[8] = {
    static_cast<uint8_t>(0x20u | (position[0] >> 3)),
    static_cast<uint8_t>((position[0] << 5) | (position[1] >> 3)),
    static_cast<uint8_t>((position[1] << 5) | (position[2] >> 3)),
    static_cast<uint8_t>((position[2] << 5) | (position[3] >> 3)),
    static_cast<uint8_t>((position[3] << 5) | (velocity >> 10)),
    static_cast<uint8_t>((velocity & 0x03FCu) >> 2),
    static_cast<uint8_t>(((velocity & 0x0003u) << 6) | (current >> 6)),
    static_cast<uint8_t>(((current & 0x003Fu) << 2) | kReportMessage1),
  };

  printPayload("Expected position payload: ", data, sizeof(data));
}

void printCanfdPositionPayload(const Options & options)
{
  uint8_t data[14] = {};
  data[0] = 0x12;
  writeFloatBe(&data[1], static_cast<float>(radToDeg(options.position_rad)));
  writeFloatBe(&data[5], static_cast<float>(options.velocity_rad_s));
  writeFloatBe(&data[9], static_cast<float>(options.current_limit_a));
  data[13] = 0;

  printPayload("Expected CAN FD position payload: ", data, sizeof(data));
}

void printCan20VelocityPayload(const Options & options)
{
  const uint16_t current = static_cast<uint16_t>(
    clampRoundToUint(options.current_limit_a * 10.0, 0.0, 3000.0));

  uint8_t data[8] = {};
  data[0] = 0x41;
  writeFloatBe(&data[1], static_cast<float>(radPerSecToRpm(options.velocity_rad_s)));
  data[5] = static_cast<uint8_t>((current >> 8) & 0xFFu);
  data[6] = static_cast<uint8_t>(current & 0xFFu);
  data[7] = 0xFF;

  printPayload("Expected velocity payload: ", data, sizeof(data));
}

void printCanfdVelocityPayload(const Options & options)
{
  uint8_t data[10] = {};
  data[0] = 0x13;
  writeFloatBe(&data[1], static_cast<float>(options.velocity_rad_s));
  writeFloatBe(&data[5], static_cast<float>(options.current_limit_a));
  data[9] = 0;

  printPayload("Expected CAN FD velocity payload: ", data, sizeof(data));
}

void printExpectedPayload(const Options & options)
{
  if (options.is_canfd) {
    if (options.mode == ControlMode::Position) {
      printCanfdPositionPayload(options);
    } else if (options.mode == ControlMode::Velocity) {
      printCanfdVelocityPayload(options);
    } else {
      printCanfdHybridPayload(options);
    }
  } else {
    if (options.mode == ControlMode::Position) {
      printCan20PositionPayload(options);
    } else if (options.mode == ControlMode::Velocity) {
      printCan20VelocityPayload(options);
    } else {
      printCan20HybridPayload(options);
    }
  }
}

bool parseArgs(int argc, char ** argv, Options & options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto needValue = [&](const char * name) -> const char * {
      if (i + 1 >= argc) {
        std::cerr << name << " requires a value\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help") {
      printUsage(argv[0]);
      return false;
    } else if (arg == "--list-models") {
      options.list_models = true;
    } else if (arg == "--canfd") {
      options.is_canfd = true;
    } else if (arg == "--dryrun") {
      options.dryrun = true;
    } else if (arg == "--can") {
      const char * value = needValue("--can");
      if (!value) {
        return false;
      }
      options.can_interface = value;
    } else if (arg == "--model") {
      const char * value = needValue("--model");
      if (!value) {
        return false;
      }
      options.model = value;
    } else if (arg == "--mode") {
      const char * value = needValue("--mode");
      if (!value) {
        return false;
      }
      const std::string mode = value;
      if (mode == "hybrid") {
        options.mode = ControlMode::Hybrid;
      } else if (mode == "position") {
        options.mode = ControlMode::Position;
      } else if (mode == "velocity") {
        options.mode = ControlMode::Velocity;
      } else {
        std::cerr << "Invalid --mode value, expected hybrid, position or velocity\n";
        return false;
      }
    } else if (arg == "--id") {
      const char * value = needValue("--id");
      if (!value || !parseU8(value, options.motor_id)) {
        std::cerr << "Invalid --id value\n";
        return false;
      }
    } else if (arg == "--pos") {
      const char * value = needValue("--pos");
      if (!value) {
        return false;
      }
      options.position_rad = std::strtod(value, nullptr);
    } else if (arg == "--vel") {
      const char * value = needValue("--vel");
      if (!value) {
        return false;
      }
      options.velocity_rad_s = std::strtod(value, nullptr);
    } else if (arg == "--current-limit") {
      const char * value = needValue("--current-limit");
      if (!value) {
        return false;
      }
      options.current_limit_a = std::strtod(value, nullptr);
    } else if (arg == "--kp") {
      const char * value = needValue("--kp");
      if (!value) {
        return false;
      }
      options.kp = std::strtod(value, nullptr);
    } else if (arg == "--kd") {
      const char * value = needValue("--kd");
      if (!value) {
        return false;
      }
      options.kd = std::strtod(value, nullptr);
    } else if (arg == "--torque") {
      const char * value = needValue("--torque");
      if (!value) {
        return false;
      }
      options.torque_nm = std::strtod(value, nullptr);
    } else if (arg == "--cycles") {
      const char * value = needValue("--cycles");
      if (!value) {
        return false;
      }
      options.cycles = std::atoi(value);
    } else if (arg == "--period-ms") {
      const char * value = needValue("--period-ms");
      if (!value) {
        return false;
      }
      options.period_ms = std::atoi(value);
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      printUsage(argv[0]);
      return false;
    }
  }

  if (options.cycles < 1 || options.period_ms < 1) {
    std::cerr << "--cycles and --period-ms must be positive\n";
    return false;
  }
  if (options.current_limit_a < 0.0) {
    std::cerr << "--current-limit must be non-negative\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parseArgs(argc, argv, options)) {
    return 1;
  }

  if (options.list_models) {
    for (const auto & model : easyarm_can::builtinMotorModels()) {
      if (model.find("xhumanoid_") == 0) {
        std::cout << model << "\n";
      }
    }
    return 0;
  }

  std::cout << "XHumanoid " << (options.is_canfd ? "CAN FD" : "CAN 2.0B")
            << " motor on " << options.can_interface
            << ", motor_id=" << static_cast<int>(options.motor_id)
            << ", model=" << options.model
            << ", mode="
            << (options.mode == ControlMode::Position ? "position" :
              options.mode == ControlMode::Velocity ? "velocity" : "hybrid")
            << ", is_canfd=" << (options.is_canfd ? "true" : "false") << "\n";
  if (options.mode == ControlMode::Position) {
    std::cout << "Position command: pos=" << options.position_rad
              << " rad, vel=" << options.velocity_rad_s
              << " rad/s, current_limit=" << options.current_limit_a
              << " A, period=" << options.period_ms
              << " ms, cycles=" << options.cycles << "\n";
  } else if (options.mode == ControlMode::Velocity) {
    std::cout << "Velocity command: vel=" << options.velocity_rad_s
              << " rad/s, current_limit=" << options.current_limit_a
              << " A, period=" << options.period_ms
              << " ms, cycles=" << options.cycles << "\n";
  } else {
    std::cout << "Hybrid command: kp=" << options.kp
              << ", kd=" << options.kd
              << ", pos=" << options.position_rad
              << " rad, vel=" << options.velocity_rad_s
              << " rad/s, torque=" << options.torque_nm
              << " Nm, period=" << options.period_ms
              << " ms, cycles=" << options.cycles << "\n";
  }
  printExpectedPayload(options);

  if (options.dryrun) {
    std::cout << "Dry run only. No CAN frames were sent.\n";
    return 0;
  }

  easyarm_can::EasyArmCan driver(options.can_interface, 0x00, options.is_canfd);
  driver.setVerbose(true);

  if (!driver.init()) {
    std::cerr << "Failed to init " << options.can_interface << ": " << driver.lastError() << "\n";
    return 1;
  }

  if (!driver.configureMotor({options.motor_id, options.model})) {
    std::cerr << "Failed to configure motor: " << driver.lastError() << "\n";
    return 1;
  }

  driver.startReceiveThread();

  if (options.is_canfd && !driver.enableMotor(options.motor_id)) {
    std::cerr << "Failed to enable motor: " << driver.lastError() << "\n";
    driver.stopReceiveThread();
    return 1;
  }

  for (int i = 0; i < options.cycles; ++i) {
    if (options.mode == ControlMode::Position) {
      easyarm_can::PositionCommand command;
      command.position_rad = options.position_rad;
      command.velocity_rad_s = options.velocity_rad_s;
      command.current_limit_a = options.current_limit_a;
      if (!driver.sendPositionControl(options.motor_id, command)) {
        std::cerr << "Failed to send position command: " << driver.lastError() << "\n";
        driver.stopReceiveThread();
        return 1;
      }
    } else if (options.mode == ControlMode::Velocity) {
      easyarm_can::VelocityCommand command;
      command.velocity_rad_s = options.velocity_rad_s;
      command.current_limit_a = options.current_limit_a;
      if (!driver.sendVelocityControl(options.motor_id, command)) {
        std::cerr << "Failed to send velocity command: " << driver.lastError() << "\n";
        driver.stopReceiveThread();
        return 1;
      }
    } else {
      easyarm_can::HybridCommand command;
      command.position_rad = options.position_rad;
      command.velocity_rad_s = options.velocity_rad_s;
      command.kp = options.kp;
      command.kd = options.kd;
      command.torque_ff_nm = options.torque_nm;
      if (!driver.sendHybridControl(options.motor_id, command)) {
        std::cerr << "Failed to send hybrid command: " << driver.lastError() << "\n";
        driver.stopReceiveThread();
        return 1;
      }
    }

    const auto feedback = driver.getMotorFeedback(options.motor_id);
    if (feedback.is_valid && i % 10 == 0) {
      std::cout << "feedback pos=" << feedback.position_rad
                << " rad, vel=" << feedback.velocity_rad_s
                << " rad/s, torque=" << feedback.torque_nm
                << " Nm, temp=" << feedback.temperature_deg_c << " degC\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(options.period_ms));
  }

  if (options.is_canfd && !driver.disableMotor(options.motor_id)) {
    std::cerr << "Failed to disable motor: " << driver.lastError() << "\n";
    driver.stopReceiveThread();
    return 1;
  }

  std::cout << "Command stream stopped."
            << (options.is_canfd ? " XHumanoid CAN FD disable command sent."
                                : " XHumanoid should stop when frames stop.")
            << "\n";
  driver.stopReceiveThread();
  return 0;
}
