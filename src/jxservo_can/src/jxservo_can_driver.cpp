/**
 * @file jxservo_can_driver.cpp
 * @brief 巨蟹智能电机 CAN 通信驱动骨架实现
 */

#include "jxservo_can/jxservo_can_driver.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace jxservo_can
{

JxservoCanDriver::JxservoCanDriver(const std::string & can_interface, uint8_t host_can_id)
: can_interface_(can_interface), host_can_id_(host_can_id), socket_fd_(-1), receive_running_(false)
{
  motor_feedbacks_.resize(256);
}

JxservoCanDriver::~JxservoCanDriver()
{
  close();
}

bool JxservoCanDriver::init()
{
  if (socket_fd_ >= 0) {
    return true;
  }

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "[JxservoCanDriver] 创建 CAN socket 失败: " << std::strerror(errno) << std::endl;
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "[JxservoCanDriver] 获取接口索引失败（" << can_interface_
              << "）: " << std::strerror(errno) << std::endl;
    close();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::cerr << "[JxservoCanDriver] 绑定 CAN socket 失败: " << std::strerror(errno)
              << std::endl;
    close();
    return false;
  }

  struct timeval rcv_timeout;
  rcv_timeout.tv_sec = 0;
  rcv_timeout.tv_usec = 100000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));

  struct timeval snd_timeout;
  snd_timeout.tv_sec = 0;
  snd_timeout.tv_usec = 10000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout));

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  if (verbose_) {
    std::cout << "[JxservoCanDriver] 已在 " << can_interface_ << " 上初始化" << std::endl;
  }
  return true;
}

void JxservoCanDriver::close()
{
  stopReceiveThread();
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    if (verbose_) {
      std::cout << "[JxservoCanDriver] 已关闭 CAN 接口" << std::endl;
    }
  }
}

bool JxservoCanDriver::enableMotor(uint8_t motor_id)
{
  (void)motor_id;
  // TODO: 根据巨蟹智能协议实现电机使能帧。
  return false;
}

bool JxservoCanDriver::disableMotor(uint8_t motor_id)
{
  (void)motor_id;
  // TODO: 根据巨蟹智能协议实现电机失能帧。
  return false;
}

MotorFeedback JxservoCanDriver::getMotorFeedback(uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);

  if (motor_id < motor_feedbacks_.size()) {
    return motor_feedbacks_[motor_id];
  }

  MotorFeedback feedback;
  feedback.motor_id = motor_id;
  feedback.is_valid = false;
  return feedback;
}

void JxservoCanDriver::startReceiveThread()
{
  if (receive_running_) {
    return;
  }

  receive_running_ = true;
  receive_thread_ = std::thread(&JxservoCanDriver::receiveThreadFunc, this);
}

void JxservoCanDriver::stopReceiveThread()
{
  if (!receive_running_) {
    return;
  }

  receive_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

bool JxservoCanDriver::sendFrame(const can_frame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);

  if (socket_fd_ < 0) {
    return false;
  }

  const ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(frame));
  if (nbytes == sizeof(frame)) {
    return true;
  }

  if (verbose_) {
    std::cerr << "[JxservoCanDriver] CAN send failed: " << std::strerror(errno) << std::endl;
  }
  return false;
}

bool JxservoCanDriver::receiveFrame(can_frame & frame, int timeout_ms)
{
  if (socket_fd_ < 0) {
    return false;
  }

  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  const int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0 || (pfd.revents & POLLIN) == 0) {
    return false;
  }

  const ssize_t nbytes = ::read(socket_fd_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

void JxservoCanDriver::receiveThreadFunc()
{
  can_frame frame;
  while (receive_running_) {
    if (receiveFrame(frame, 10)) {
      parseFeedback(frame);
    }
  }
}

void JxservoCanDriver::parseFeedback(const can_frame & frame)
{
  (void)frame;
  // TODO: 根据巨蟹智能协议解析 motor_id、位置(rad)、速度(rad/s)、力矩(Nm)和温度(degC)。
  // 解析完成后应在 feedback_mutex_ 保护下写入 motor_feedbacks_[motor_id]。
}

}  // namespace jxservo_can
