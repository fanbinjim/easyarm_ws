#include "easyarm_can/socket_can_transport.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace easyarm_can
{

SocketCanTransport::SocketCanTransport(const std::string & can_interface)
: can_interface_(can_interface)
{
}

SocketCanTransport::~SocketCanTransport()
{
  close();
}

bool SocketCanTransport::init(bool enable_can_fd)
{
  if (socket_fd_ >= 0) {
    return true;
  }

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    setError(std::string("create CAN socket failed: ") + std::strerror(errno));
    return false;
  }

  if (enable_can_fd) {
    const int enable = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
      setError(std::string("enable CAN FD frames failed: ") + std::strerror(errno));
      close();
      return false;
    }
    can_fd_enabled_ = true;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    setError("get interface index failed for " + can_interface_ + ": " + std::strerror(errno));
    close();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    setError(std::string("bind CAN socket failed: ") + std::strerror(errno));
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

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  if (verbose_) {
    std::cout << "[easyarm_can] initialized " << can_interface_
              << (can_fd_enabled_ ? " with CAN FD" : "") << std::endl;
  }

  return true;
}

void SocketCanTransport::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool SocketCanTransport::send(const can_frame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (socket_fd_ < 0) {
    setError("send failed: socket is not connected");
    return false;
  }

  const ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(frame));
  if (nbytes == sizeof(frame)) {
    return true;
  }

  setError(std::string("send CAN frame failed: ") + std::strerror(errno));
  return false;
}

bool SocketCanTransport::send(const canfd_frame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (socket_fd_ < 0) {
    setError("send failed: socket is not connected");
    return false;
  }
  if (!can_fd_enabled_) {
    setError("send CAN FD frame failed: CAN FD is not enabled");
    return false;
  }

  const ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(frame));
  if (nbytes == sizeof(frame)) {
    return true;
  }

  setError(std::string("send CAN FD frame failed: ") + std::strerror(errno));
  return false;
}

bool SocketCanTransport::receive(canfd_frame & frame, int timeout_ms)
{
  if (socket_fd_ < 0) {
    setError("receive failed: socket is not connected");
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

  std::memset(&frame, 0, sizeof(frame));
  const ssize_t nbytes = ::read(socket_fd_, &frame, sizeof(frame));
  if (nbytes == CAN_MTU || nbytes == CANFD_MTU) {
    return true;
  }

  setError(std::string("receive CAN frame failed: ") + std::strerror(errno));
  return false;
}

std::string SocketCanTransport::lastError() const
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void SocketCanTransport::setError(const std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
  }
  if (verbose_) {
    std::cerr << "[easyarm_can] " << message << std::endl;
  }
}

}  // namespace easyarm_can
