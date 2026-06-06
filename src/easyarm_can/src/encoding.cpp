#include "easyarm_can/encoding.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace easyarm_can
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double clampValue(double value, double min_value, double max_value)
{
  if (min_value > max_value) {
    std::swap(min_value, max_value);
  }
  return std::min(std::max(value, min_value), max_value);
}

uint32_t floatToUnsigned(double value, double min_value, double max_value, unsigned bits)
{
  if (bits == 0 || bits >= 32 || max_value <= min_value) {
    return 0;
  }

  const double clamped = clampValue(value, min_value, max_value);
  const uint32_t raw_max = (1u << bits) - 1u;
  const double scaled = (clamped - min_value) * static_cast<double>(raw_max) /
    (max_value - min_value);
  return static_cast<uint32_t>(std::lround(scaled));
}

double unsignedToFloat(uint32_t value, double min_value, double max_value, unsigned bits)
{
  if (bits == 0 || bits >= 32 || max_value <= min_value) {
    return min_value;
  }

  const uint32_t raw_max = (1u << bits) - 1u;
  const uint32_t clamped = std::min(value, raw_max);
  return static_cast<double>(clamped) * (max_value - min_value) /
    static_cast<double>(raw_max) + min_value;
}

void writeU16Be(uint8_t * data, uint16_t value)
{
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  data[1] = static_cast<uint8_t>(value & 0xFFu);
}

void writeI16Be(uint8_t * data, int16_t value)
{
  writeU16Be(data, static_cast<uint16_t>(value));
}

void writeU16Le(uint8_t * data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFFu);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void writeI16Le(uint8_t * data, int16_t value)
{
  writeU16Le(data, static_cast<uint16_t>(value));
}

void writeI32Le(uint8_t * data, int32_t value)
{
  const uint32_t raw = static_cast<uint32_t>(value);
  data[0] = static_cast<uint8_t>(raw & 0xFFu);
  data[1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
  data[2] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
  data[3] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
}

void writeFloatBe(uint8_t * data, float value)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "float must be IEEE-754 32-bit");
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  data[0] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
  data[1] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
  data[2] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
  data[3] = static_cast<uint8_t>(raw & 0xFFu);
}

uint16_t readU16Be(const uint8_t * data)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

int16_t readI16Be(const uint8_t * data)
{
  return static_cast<int16_t>(readU16Be(data));
}

uint16_t readU16Le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

int16_t readI16Le(const uint8_t * data)
{
  return static_cast<int16_t>(readU16Le(data));
}

int32_t readI32Le(const uint8_t * data)
{
  const uint32_t raw =
    static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
  return static_cast<int32_t>(raw);
}

float readFloatBe(const uint8_t * data)
{
  const uint32_t raw =
    (static_cast<uint32_t>(data[0]) << 24) |
    (static_cast<uint32_t>(data[1]) << 16) |
    (static_cast<uint32_t>(data[2]) << 8) |
    static_cast<uint32_t>(data[3]);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

double radToDeg(double rad)
{
  return rad * 180.0 / kPi;
}

double degToRad(double deg)
{
  return deg * kPi / 180.0;
}

double radPerSecToRpm(double rad_s)
{
  return rad_s * 60.0 / (2.0 * kPi);
}

double rpmToRadPerSec(double rpm)
{
  return rpm * 2.0 * kPi / 60.0;
}

}  // namespace easyarm_can
