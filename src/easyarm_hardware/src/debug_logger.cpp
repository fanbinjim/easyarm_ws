#include "easyarm_hardware/debug_logger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

namespace easyarm_hardware
{

namespace
{

constexpr char kDebugLogMagic[8] = {'E', 'A', 'H', 'D', 'B', 'G', '1', '\0'};
constexpr uint32_t kDebugLogVersion = 1;
constexpr size_t kBatchSize = 64;

size_t next_index(size_t index, size_t capacity)
{
  return index + 1 == capacity ? 0 : index + 1;
}

}  // namespace

DebugLogger::~DebugLogger()
{
  stop();
}

bool DebugLogger::start(const DebugLoggerConfig & config)
{
  stop();
  config_ = config;
  if (!config_.enabled) {
    return true;
  }

  const double buffer_seconds = std::max(config_.buffer_seconds, 1.0);
  const double sample_rate_hz = std::max(config_.sample_rate_hz, 1.0);
  capacity_ = static_cast<size_t>(std::ceil(buffer_seconds * sample_rate_hz)) + 1;
  capacity_ = std::max<size_t>(capacity_, 16);

  buffer_.assign(capacity_, HardwareDebugSample{});
  head_.store(0, std::memory_order_relaxed);
  tail_.store(0, std::memory_order_relaxed);
  dropped_count_.store(0, std::memory_order_relaxed);
  written_count_.store(0, std::memory_order_relaxed);

  file_.open(config_.path, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!file_.is_open()) {
    buffer_.clear();
    capacity_ = 0;
    return false;
  }

  if (!write_header()) {
    file_.close();
    buffer_.clear();
    capacity_ = 0;
    return false;
  }

  active_.store(true, std::memory_order_release);
  thread_ = std::thread(&DebugLogger::run, this);
  return true;
}

void DebugLogger::stop()
{
  if (!active_.exchange(false, std::memory_order_acq_rel)) {
    if (thread_.joinable()) {
      thread_.join();
    }
    if (file_.is_open()) {
      file_.flush();
      file_.close();
    }
    return;
  }

  if (thread_.joinable()) {
    thread_.join();
  }
  if (file_.is_open()) {
    file_.flush();
    file_.close();
  }
}

bool DebugLogger::push(const HardwareDebugSample & sample)
{
  if (!active_.load(std::memory_order_acquire) || capacity_ == 0) {
    return true;
  }

  const size_t head = head_.load(std::memory_order_relaxed);
  const size_t next = next_index(head, capacity_);
  if (next == tail_.load(std::memory_order_acquire)) {
    dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  buffer_[head] = sample;
  head_.store(next, std::memory_order_release);
  return true;
}

uint64_t DebugLogger::dropped_count() const
{
  return dropped_count_.load(std::memory_order_relaxed);
}

uint64_t DebugLogger::written_count() const
{
  return written_count_.load(std::memory_order_relaxed);
}

bool DebugLogger::is_active() const
{
  return active_.load(std::memory_order_acquire);
}

bool DebugLogger::pop(HardwareDebugSample & sample)
{
  const size_t tail = tail_.load(std::memory_order_relaxed);
  if (tail == head_.load(std::memory_order_acquire)) {
    return false;
  }

  sample = buffer_[tail];
  tail_.store(next_index(tail, capacity_), std::memory_order_release);
  return true;
}

void DebugLogger::run()
{
  std::array<HardwareDebugSample, kBatchSize> batch{};
  while (active_.load(std::memory_order_acquire) ||
    tail_.load(std::memory_order_acquire) != head_.load(std::memory_order_acquire))
  {
    size_t count = 0;
    while (count < batch.size() && pop(batch[count])) {
      ++count;
    }

    if (count > 0 && file_.is_open()) {
      file_.write(
        reinterpret_cast<const char *>(batch.data()),
        static_cast<std::streamsize>(count * sizeof(HardwareDebugSample)));
      written_count_.fetch_add(count, std::memory_order_relaxed);
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

bool DebugLogger::write_header()
{
  DebugLogFileHeader header{};
  std::memcpy(header.magic, kDebugLogMagic, sizeof(header.magic));
  header.version = kDebugLogVersion;
  header.header_size = static_cast<uint32_t>(sizeof(DebugLogFileHeader));
  header.sample_size = static_cast<uint32_t>(sizeof(HardwareDebugSample));
  header.joint_count = static_cast<uint32_t>(kDebugJointCount);
  header.start_steady_time_ns = steady_time_ns();
  header.start_system_time_ns = system_time_ns();

  file_.write(reinterpret_cast<const char *>(&header), sizeof(header));
  return file_.good();
}

int64_t DebugLogger::steady_time_ns()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

int64_t DebugLogger::system_time_ns()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

}  // namespace easyarm_hardware
