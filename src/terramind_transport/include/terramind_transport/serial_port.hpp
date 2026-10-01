#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace terramind::transport
{
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort()
  {
    close();
  }
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  void open(const std::string & path, int baud = 115200);
  void close();
  void discard_input();
  bool is_open() const
  {
    return fd_ >= 0;
  }
  std::vector<uint8_t> read_some(int timeout_ms = 5);
  void write_all(const std::vector<uint8_t> & bytes, int timeout_ms = 15);

private:
  int fd_ = -1;
};
}
