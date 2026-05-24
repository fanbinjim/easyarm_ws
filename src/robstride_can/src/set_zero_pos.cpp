#include "robstride_can/robstride_can_driver.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void printUsage(const char * program_name)
{
  std::cout << "Usage: " << program_name << " <can_interface> <motor_id>\n"
            << "Example: " << program_name << " can0 0x01\n"
            << "         " << program_name << " can0 1\n\n"
            << "Arguments:\n"
            << "  can_interface  CAN interface name (e.g., can0, can1, can4)\n"
            << "  motor_id       Motor ID in hex (e.g., 0x01) or decimal (e.g., 1)\n";
}

uint8_t parseMotorId(const std::string & value)
{
  size_t parsed_chars = 0;
  const unsigned long parsed = std::stoul(value, &parsed_chars, 0);
  if (parsed_chars != value.size() || parsed > 0xFF) {
    throw std::invalid_argument("motor_id must be in range 0..255");
  }
  return static_cast<uint8_t>(parsed);
}

void printHeader(const std::string & can_interface, uint8_t motor_id)
{
  std::cout << "========================================\n"
            << "  Motor Zero Position Setting Tool\n"
            << "========================================\n"
            << "CAN Interface: " << can_interface << "\n"
            << "Motor ID:      0x" << std::hex << std::uppercase << std::setw(2)
            << std::setfill('0') << static_cast<int>(motor_id)
            << std::dec << std::nouppercase << std::setfill(' ') << "\n"
            << "========================================\n\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  using robstride_can::RobstrideCanDriver;

  if (argc != 3) {
    std::cerr << "Error: Invalid number of arguments\n";
    printUsage(argv[0]);
    return 2;
  }

  const std::string can_interface = argv[1];
  uint8_t motor_id = 0;

  try {
    motor_id = parseMotorId(argv[2]);
  } catch (const std::exception & error) {
    std::cerr << "Argument error: " << error.what() << "\n";
    printUsage(argv[0]);
    return 2;
  }

  printHeader(can_interface, motor_id);

  RobstrideCanDriver driver(can_interface);

  std::cout << "[1/2] Initializing CAN driver...\n";
  if (!driver.init()) {
    std::cerr << "Error: Failed to init CAN driver on " << can_interface << "\n";
    return 1;
  }
  std::cout << "      CAN driver initialized successfully\n\n";

  std::cout << "[2/2] Setting zero position...\n";
  if (!driver.setZeroPosition(motor_id)) {
    std::cerr << "Error: Failed to send zero position command\n";
    driver.close();
    return 1;
  }
  std::cout << "      Zero position command sent successfully\n\n";

  std::cout << "========================================\n"
            << "  Zero position setting completed!\n"
            << "========================================\n";

  driver.close();
  return 0;
}
