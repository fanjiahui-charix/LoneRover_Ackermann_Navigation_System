#include "origincarpro_base/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>

namespace origincarpro_base
{

SerialPort::SerialPort()
: fd_(-1)
{
}

SerialPort::~SerialPort()
{
  closePort();
}

bool SerialPort::openPort(const std::string & port_name, int baud_rate)
{
  closePort();

  fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    closePort();
    return false;
  }

  cfmakeraw(&tty);

  const int baud = toTermiosBaud(baud_rate);
  if (baud < 0) {
    closePort();
    return false;
  }

  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    closePort();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::closePort()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const
{
  return fd_ >= 0;
}

int SerialPort::readBytes(uint8_t * data, std::size_t max_len)
{
  if (fd_ < 0) {
    return -1;
  }

  const int n = ::read(fd_, data, max_len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    return -1;
  }

  return n;
}

bool SerialPort::writeBytes(const uint8_t * data, std::size_t len)
{
  if (fd_ < 0) {
    return false;
  }

  std::size_t written = 0;
  while (written < len) {
    const int n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(n);
  }

  return true;
}

int SerialPort::toTermiosBaud(int baud_rate) const
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return -1;
  }
}

}  // namespace origincarpro_base
