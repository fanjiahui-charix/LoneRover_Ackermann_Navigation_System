#include "origincarpro_base/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace origincarpro_base
{

uint16_t crc16CcittFalse(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0xFFFF;

  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

namespace
{

void parseLegacyPayload(const uint8_t * payload, RobotFrame & out)
{
  out.flag_stop = payload[0];

  out.vx_mps = static_cast<double>(int16FromBigEndian(payload[1], payload[2])) / 1000.0;
  out.vy_mps = static_cast<double>(int16FromBigEndian(payload[3], payload[4])) / 1000.0;
  out.wz_radps = static_cast<double>(int16FromBigEndian(payload[5], payload[6])) / 1000.0;

  out.acc_x_raw = int16FromBigEndian(payload[7], payload[8]);
  out.acc_y_raw = int16FromBigEndian(payload[9], payload[10]);
  out.acc_z_raw = int16FromBigEndian(payload[11], payload[12]);

  out.gyro_x_raw = int16FromBigEndian(payload[13], payload[14]);
  out.gyro_y_raw = int16FromBigEndian(payload[15], payload[16]);
  out.gyro_z_raw = int16FromBigEndian(payload[17], payload[18]);

  out.voltage_mv = int16FromBigEndian(payload[19], payload[20]);
}

}  // namespace

bool parseRobotFrameLegacy(const std::array<uint8_t, ROBOT_RX_LEGACY_LEN> & buf, RobotFrame & out)
{
  // 24-byte frame: [0x7B][flag][vx_H][vx_L][vy_H][vy_L][vz_H][vz_L]
  //                [ax_H][ax_L][ay_H][ay_L][az_H][az_L]
  //                [gx_H][gx_L][gy_H][gy_L][gz_H][gz_L]
  //                [volt_H][volt_L][xor][0x7D]
  if (buf[0] != FRAME_HEADER || buf[ROBOT_RX_LEGACY_LEN - 1] != FRAME_TAIL) {
    return false;
  }

  if (xorChecksum(buf.data(), 22) != buf[22]) {
    return false;
  }

  out = RobotFrame{};
  out.protocol_version = 1;
  parseLegacyPayload(&buf[1], out);

  return true;
}

bool parseRobotFrameV2(const std::array<uint8_t, ROBOT_RX_V2_LEN> & buf, RobotFrame & out)
{
  // 34-byte frame:
  // [0] header
  // [1] protocol_version, [2] frame_type, [3] payload_len
  // [4..5] seq, [6..9] tick_ms
  // [10..30] legacy payload without header/checksum/tail
  // [31..32] CRC16-CCITT-FALSE over bytes [1..30], [33] tail
  if (buf[0] != FRAME_HEADER || buf[ROBOT_RX_V2_LEN - 1] != FRAME_TAIL) {
    return false;
  }

  if (buf[1] != ROBOT_RX_PROTOCOL_V2 ||
    buf[2] != ROBOT_RX_FRAME_TYPE_ODOM_IMU ||
    buf[3] != ROBOT_RX_V2_PAYLOAD_LEN)
  {
    return false;
  }

  const uint16_t expected_crc = uint16FromBigEndian(buf[31], buf[32]);
  if (crc16CcittFalse(&buf[1], 30) != expected_crc) {
    return false;
  }

  out = RobotFrame{};
  out.has_protocol_timing = true;
  out.protocol_version = buf[1];
  out.frame_type = buf[2];
  out.payload_len = buf[3];
  out.seq = uint16FromBigEndian(buf[4], buf[5]);
  out.tick_ms = uint32FromBigEndian(buf[6], buf[7], buf[8], buf[9]);

  parseLegacyPayload(&buf[10], out);

  return true;
}

std::array<uint8_t, ROS_TX_LEN> buildCmdVelFrame(double vx_mps, double vy_mps, double wz_radps)
{
  std::array<uint8_t, ROS_TX_LEN> buf {};
  const auto clamp_i16 = [](double value) {
    value = std::max(-32768.0, std::min(32767.0, value));
    return static_cast<int16_t>(std::lround(value));
  };

  const int16_t vx = clamp_i16(vx_mps * 1000.0);
  const int16_t vy = clamp_i16(vy_mps * 1000.0);
  const int16_t wz = clamp_i16(wz_radps * 1000.0);

  buf[0] = FRAME_HEADER;
  buf[1] = 0;
  buf[2] = 0;

  int16ToBigEndian(vx, buf[3], buf[4]);
  int16ToBigEndian(vy, buf[5], buf[6]);
  int16ToBigEndian(wz, buf[7], buf[8]);

  buf[9] = xorChecksum(buf.data(), 9);
  buf[10] = FRAME_TAIL;

  return buf;
}

}  // namespace origincarpro_base
