// 仅使用 PTY 验证字节读写和设备锁，不访问物理串口。
#include "terramind_transport/serial_port.hpp"
#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>
#include <poll.h>
TEST(Transport, PtyReadWriteAndOwnership) {
  int master, slave;
  char path[256];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  terramind::transport::SerialPort p;
  p.open(path);
  terramind::transport::SerialPort second;
  EXPECT_THROW(second.open(path), std::runtime_error);
  p.write_all({0xfc, 0, 0xff});
  pollfd fd{master, POLLIN, 0};
  ASSERT_GT(poll(&fd, 1, 100), 0);
  uint8_t b[3];
  ASSERT_EQ(read(master, b, 3), 3);
  EXPECT_EQ(b[2], 255);
  ASSERT_EQ(write(master, b, 3), 3);
  auto out = p.read_some(100);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], 0xfc);
  p.close();
  EXPECT_NO_THROW(second.open(path));
  second.close();
  close(master);
  close(slave);
}
TEST(Transport, InvalidDeviceAndBaud) {
  terramind::transport::SerialPort p;
  EXPECT_THROW(p.open("/not/a/serial/port"), std::runtime_error);
  EXPECT_THROW(p.open("/dev/null", 9600), std::invalid_argument);
}
