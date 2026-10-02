#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace terramind::transport
{
// Linux 串口/PTY 的 RAII 封装；只传输字节，不解析协议，不负责重连。
// 对象不可复制，析构关闭句柄；同一个实例的读写由调用者串行执行。
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
  // 超时或暂时无数据返回空数组；断开和不可恢复错误抛异常。
  std::vector<uint8_t> read_some(int timeout_ms = 5);
  // 必须在整个调用期限内写完；失败后调用者应关闭连接，避免续写残帧。
  void write_all(const std::vector<uint8_t> & bytes, int timeout_ms = 15);

private:
  int fd_ = -1;
};
}
