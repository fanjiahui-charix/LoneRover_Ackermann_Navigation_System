#ifndef ORIGINCARPRO_BASE_PROTOCOL_HPP_
#define ORIGINCARPRO_BASE_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace origincarpro_base
{

constexpr uint8_t FRAME_HEADER = 0x7B;
constexpr uint8_t FRAME_TAIL = 0x7D;

constexpr std::size_t ROBOT_RX_LEGACY_LEN = 24;
constexpr std::size_t ROBOT_RX_V2_LEN = 34;
constexpr uint8_t ROBOT_RX_PROTOCOL_V2 = 2;
constexpr uint8_t ROBOT_RX_FRAME_TYPE_ODOM_IMU = 1;
constexpr uint8_t ROBOT_RX_V2_PAYLOAD_LEN = 21;
constexpr std::size_t ROS_TX_LEN = 11;

struct RobotFrame
{
  bool has_protocol_timing = false;
  uint8_t protocol_version = 1;
  uint8_t frame_type = 0;
  uint8_t payload_len = 0;
  uint16_t seq = 0;
  uint32_t tick_ms = 0;

  uint8_t flag_stop = 0;

  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double wz_radps = 0.0;

  int16_t acc_x_raw = 0;
  int16_t acc_y_raw = 0;
  int16_t acc_z_raw = 0;

  int16_t gyro_x_raw = 0;
  int16_t gyro_y_raw = 0;
  int16_t gyro_z_raw = 0;

  int16_t voltage_mv = 0;
};

inline int16_t int16FromBigEndian(uint8_t high, uint8_t low)
{
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

inline void int16ToBigEndian(int16_t value, uint8_t & high, uint8_t & low)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  high = static_cast<uint8_t>((raw >> 8) & 0xFF);
  low = static_cast<uint8_t>(raw & 0xFF);
}

inline uint8_t xorChecksum(const uint8_t * data, std::size_t len)
{
  uint8_t check = 0;
  for (std::size_t i = 0; i < len; ++i) {
    check ^= data[i];
  }
  return check;
}

inline uint16_t uint16FromBigEndian(uint8_t high, uint8_t low)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

inline uint32_t uint32FromBigEndian(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  return (static_cast<uint32_t>(b0) << 24) |
         (static_cast<uint32_t>(b1) << 16) |
         (static_cast<uint32_t>(b2) << 8) |
         static_cast<uint32_t>(b3);
}

uint16_t crc16CcittFalse(const uint8_t * data, std::size_t len);

bool parseRobotFrameLegacy(const std::array<uint8_t, ROBOT_RX_LEGACY_LEN> & buf, RobotFrame & out);
bool parseRobotFrameV2(const std::array<uint8_t, ROBOT_RX_V2_LEN> & buf, RobotFrame & out);
std::array<uint8_t, ROS_TX_LEN> buildCmdVelFrame(double vx_mps, double vy_mps, double wz_radps);

}  // namespace origincarpro_base

#endif  // ORIGINCARPRO_BASE_PROTOCOL_HPP_
