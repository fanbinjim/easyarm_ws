/**
 * @file socket_can_transport.hpp
 * @brief SocketCAN/CAN FD 传输层。
 */

#ifndef EASYARM_CAN__SOCKET_CAN_TRANSPORT_HPP_
#define EASYARM_CAN__SOCKET_CAN_TRANSPORT_HPP_

#include <mutex>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace easyarm_can
{

class SocketCanTransport
{
public:
  explicit SocketCanTransport(const std::string & can_interface);
  ~SocketCanTransport();

  bool init(bool enable_can_fd);
  void close();
  bool isConnected() const { return socket_fd_ >= 0; }
  void setVerbose(bool verbose) { verbose_ = verbose; }

  bool send(const can_frame & frame);
  bool send(const canfd_frame & frame);
  bool receive(canfd_frame & frame, int timeout_ms);

  std::string lastError() const;

private:
  void setError(const std::string & message);

  std::string can_interface_;
  int socket_fd_{-1};
  bool verbose_{true};
  bool can_fd_enabled_{false};
  mutable std::mutex send_mutex_;
  mutable std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace easyarm_can

#endif  // EASYARM_CAN__SOCKET_CAN_TRANSPORT_HPP_
