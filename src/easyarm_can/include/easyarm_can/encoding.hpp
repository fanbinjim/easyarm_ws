/**
 * @file encoding.hpp
 * @brief CAN 协议端序、单位和线性量化工具。
 */

#ifndef EASYARM_CAN__ENCODING_HPP_
#define EASYARM_CAN__ENCODING_HPP_

#include <cstddef>
#include <cstdint>

namespace easyarm_can
{

double clampValue(double value, double min_value, double max_value);
uint32_t floatToUnsigned(double value, double min_value, double max_value, unsigned bits);
double unsignedToFloat(uint32_t value, double min_value, double max_value, unsigned bits);

void writeU16Be(uint8_t * data, uint16_t value);
void writeI16Be(uint8_t * data, int16_t value);
void writeU16Le(uint8_t * data, uint16_t value);
void writeI16Le(uint8_t * data, int16_t value);
void writeI32Le(uint8_t * data, int32_t value);
void writeFloatBe(uint8_t * data, float value);

uint16_t readU16Be(const uint8_t * data);
int16_t readI16Be(const uint8_t * data);
uint16_t readU16Le(const uint8_t * data);
int16_t readI16Le(const uint8_t * data);
int32_t readI32Le(const uint8_t * data);
float readFloatBe(const uint8_t * data);

double radToDeg(double rad);
double degToRad(double deg);
double radPerSecToRpm(double rad_s);
double rpmToRadPerSec(double rpm);

}  // namespace easyarm_can

#endif  // EASYARM_CAN__ENCODING_HPP_
