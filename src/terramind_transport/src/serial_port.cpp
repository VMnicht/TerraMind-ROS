#include "terramind_transport/serial_port.hpp"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>
namespace terramind::transport
{
namespace
{
std::runtime_error error(const char * operation)
{
  return std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
}
void SerialPort::open(const std::string & path, int baud)
{
  close();
  speed_t speed;
  switch (baud) {
    case 115200: speed = B115200; break;
    case 460800: speed = B460800; break;
    default: throw std::invalid_argument("supported serial baud: 115200 or 460800");
  }
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw error("open serial");
  }
  try {
    if (flock(fd_, LOCK_EX | LOCK_NB) < 0) {
      // 建议锁防止遵守同一约定的驱动/发现器同时消费一个串口的数据。
      throw error("serial already owned");
    }
    termios settings{};
    if (tcgetattr(fd_, &settings) < 0) {
      throw error("tcgetattr");
    }
    cfmakeraw(&settings);
    // 两种板卡共用 8N1、无硬件流控；原始模式禁止回显和字节转换。
    if (cfsetispeed(&settings, speed) < 0 || cfsetospeed(&settings, speed) < 0) {
      throw error("set serial speed");
    }
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &settings) < 0) {
      throw error("tcsetattr");
    }
    if (tcflush(fd_, TCIOFLUSH) < 0) {
      throw error("tcflush");
    }
  } catch (...) {
    close();
    throw;
  }
}
void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}
void SerialPort::discard_input()
{
  if (fd_ < 0) {
    throw std::runtime_error("serial closed");
  }
  if (tcflush(fd_, TCIFLUSH) < 0) {
    throw error("flush serial input");
  }
}
std::vector<uint8_t> SerialPort::read_some(int timeout_ms)
{
  if (fd_ < 0) {
    throw std::runtime_error("serial closed");
  }
  pollfd p{fd_, POLLIN, 0};
  int r = poll(&p, 1, timeout_ms);
  if (r < 0) {
    if (errno == EINTR) {
      return std::vector<uint8_t>{};
    }
    throw error("poll serial");
  }
  if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error("serial disconnected");
  }
  if (!(p.revents & POLLIN)) {
    return std::vector<uint8_t>{};
  }
  std::vector<uint8_t> out(512);
  auto n = ::read(fd_, out.data(), out.size());
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR) {
      return std::vector<uint8_t>{};
    }
    throw error("read serial");
  }
  if (n == 0) {
    throw std::runtime_error("serial EOF");
  }
  out.resize(n);
  return out;
}
void SerialPort::write_all(const std::vector<uint8_t> & b, int timeout_ms)
{
  if (fd_ < 0) {
    throw std::runtime_error("serial closed");
  }
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  // 部分写入、EINTR 重试共享同一截止时间，不能因重试无限延长发送。
  size_t offset = 0;
  while (offset < b.size()) {
    auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remain <= 0) {
      throw std::runtime_error("serial write deadline exceeded");
    }
    pollfd p{fd_, POLLOUT, 0};
    int r = poll(&p, 1, static_cast<int>(remain));
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw error("poll write");
    }
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error("serial disconnected during write");
    }
    if (!(p.revents & POLLOUT)) {
      continue;
    }
    auto n = ::write(fd_, b.data() + offset, b.size() - offset);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        continue;
      }
      throw error("write serial");
    }
    offset += n;
  }
}
}
