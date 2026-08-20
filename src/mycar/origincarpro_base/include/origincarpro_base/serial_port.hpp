#ifndef ORIGINCARPRO_BASE_SERIAL_PORT_HPP_
#define ORIGINCARPRO_BASE_SERIAL_PORT_HPP_

#include <cstdint>
#include <string>

namespace origincarpro_base
{

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  bool openPort(const std::string & port_name, int baud_rate);
  void closePort();
  bool isOpen() const;

  int readBytes(uint8_t * data, std::size_t max_len);
  bool writeBytes(const uint8_t * data, std::size_t len);

private:
  int fd_;
  int toTermiosBaud(int baud_rate) const;
};

}  // namespace origincarpro_base

#endif  // ORIGINCARPRO_BASE_SERIAL_PORT_HPP_
