#include "ti5robot_can/ti5robot_can_driver.hpp"

#include <iostream>
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

namespace ti5robot_can
{

Ti5robotCanDriver::Ti5robotCanDriver(const std::string& can_interface, uint8_t host_can_id)
  : can_interface_(can_interface), host_can_id_(host_can_id), socket_fd_(-1), receive_running_(false)
{
}

Ti5robotCanDriver::~Ti5robotCanDriver()
{
  close();
}

bool Ti5robotCanDriver::init()
{
  if (socket_fd_ >= 0) {
    return true;
  }

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "Failed to create CAN socket" << std::endl;
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Failed to get interface index for " << can_interface_ << std::endl;
    close();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "Failed to bind CAN socket" << std::endl;
    close();
    return false;
  }

  // 设置接收超时
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100ms
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  return true;
}

void Ti5robotCanDriver::close()
{
  if (socket_fd_ >= 0) {
    stopReceiveThread();
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool Ti5robotCanDriver::enableMotor(uint8_t motor_id)
{
  // TODO: 实现钛虎电机使能协议
  (void)motor_id;
  return false;
}

bool Ti5robotCanDriver::disableMotor(uint8_t motor_id)
{
  // TODO: 实现钛虎电机失能协议
  (void)motor_id;
  return false;
}

MotorFeedback Ti5robotCanDriver::getMotorFeedback(uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (motor_id < motor_feedbacks_.size()) {
    return motor_feedbacks_[motor_id];
  }
  MotorFeedback empty;
  empty.motor_id = motor_id;
  empty.is_valid = false;
  return empty;
}

void Ti5robotCanDriver::startReceiveThread()
{
  if (receive_running_) {
    return;
  }
  receive_running_ = true;
  receive_thread_ = std::thread(&Ti5robotCanDriver::receiveThreadFunc, this);
}

void Ti5robotCanDriver::stopReceiveThread()
{
  receive_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

bool Ti5robotCanDriver::sendFrame(const can_frame& frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (socket_fd_ < 0) {
    return false;
  }
  int bytes_sent = write(socket_fd_, &frame, sizeof(frame));
  return bytes_sent == sizeof(frame);
}

bool Ti5robotCanDriver::receiveFrame(can_frame& frame, int timeout_ms)
{
  if (socket_fd_ < 0) {
    return false;
  }
  
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(socket_fd_, &read_set);
  
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  
  int ret = select(socket_fd_ + 1, &read_set, nullptr, nullptr, &timeout);
  if (ret > 0 && FD_ISSET(socket_fd_, &read_set)) {
    int bytes_read = read(socket_fd_, &frame, sizeof(frame));
    return bytes_read == sizeof(frame);
  }
  return false;
}

void Ti5robotCanDriver::receiveThreadFunc()
{
  while (receive_running_) {
    can_frame frame;
    if (receiveFrame(frame, 10)) {
      parseFeedback(frame);
    }
  }
}

void Ti5robotCanDriver::parseFeedback(const can_frame& frame)
{
  // TODO: 实现钛虎电机反馈解析协议
  (void)frame;
}

}  // namespace ti5robot_can